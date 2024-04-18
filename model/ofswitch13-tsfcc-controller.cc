/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Luciano Chaves <luciano@lrc.ic.unicamp.br>
 */

#ifdef NS3_OFSWITCH13

#include "ofswitch13-tsfcc-controller.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"

namespace ns3 {
NS_LOG_COMPONENT_DEFINE ("OFSwitch13TsfccController");
NS_OBJECT_ENSURE_REGISTERED (OFSwitch13TsfccController);

/********** Public methods ***********/
OFSwitch13TsfccController::OFSwitch13TsfccController ()
{
  NS_LOG_FUNCTION (this);
}

OFSwitch13TsfccController::~OFSwitch13TsfccController ()
{
  NS_LOG_FUNCTION (this);
}

TypeId
OFSwitch13TsfccController::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::OFSwitch13TsfccController")
    .SetParent<OFSwitch13Controller> ()
    .SetGroupName ("OFSwitch13")
    .AddConstructor<OFSwitch13TsfccController> ()
  ;
  return tid;
}
/**
 * @brief 通过定期检测“新流量”的数量来预测Incast事件
 * 
 */
void
OFSwitch13TsfccController::PredictIncast(){
  //1.查询所有端口的新流数量，并重新设置为0
  for (auto& switchData : m_newFlowNumOnSwitchMap) {
      uint64_t dpId = switchData.first;
      for (auto& portData : switchData.second) {
        uint32_t port_no = portData.first;
        if(portData.second >= IncastThreshold){
          //降低这个端口出去的所有流量的RWND
          uint16_t flow_num = 0;
          uint16_t elephant_num = 0;
          ClassifyTraffic(flow_num, elephant_num, dpId, port_no);
          if(flow_num != 0 && flow_num >= IncastThreshold){
            double fair_window = (BDP+queue_threshold_l*1500)/flow_num;
            uint16_t rwnd =std::max(int(fair_window/4), int(max_size/4));
            // NS_LOG_WARN("dpId: " << dpId << " rwnd: "  << rwnd);
            UpdateMouseRWND(rwnd*4, dpId, port_no, true);
            UpdateMouseRWND(rwnd, dpId, port_no, false);
            UpdateElephantRWND(int(max_size/4), dpId, port_no);
          }
        }
        portData.second = 0;
      }
  }
  Simulator::Schedule (Seconds (0.00006), &OFSwitch13TsfccController::PredictIncast, this);
}

void
OFSwitch13TsfccController::DoDispose ()
{
  NS_LOG_FUNCTION (this);

  m_learnedInfo.clear ();
  OFSwitch13Controller::DoDispose ();
}

/**
 * @brief 从全局流表中识别出大象流，放入大象流表中
 * 
 * @param flow_num 总流的数量
 * @param elephant_num 大象流的数量
 * @param dpId 交换机ID
 * @param port_no 交换机出端口
 */
void OFSwitch13TsfccController::ClassifyTraffic(uint16_t &flow_num, uint16_t &elephant_num, uint64_t dpId, uint32_t port_no){
  FlowTableMap_t::iterator it;
  for (it = m_globalFlowTable.begin(); it != m_globalFlowTable.end(); ++it) {
    FlowStats flow = it->second;
    std::vector<std::pair<uint64_t, SwitchInfo>>::iterator switch_it;
    for (switch_it = flow.switches.begin(); switch_it != flow.switches.end(); switch_it++) {
      if(dpId == switch_it->first && switch_it->second.out_port == port_no){
        flow_num += 1;
        Time now = Simulator::Now();
        flow.exist_time = (now - flow.start_time).GetSeconds();
        // NS_LOG_DEBUG ("------" << flow.exist_time);
        if(flow.exist_time > 0.2){
          auto eleIt = m_elephantFlowTable.find(it->first);
          if (eleIt == m_elephantFlowTable.end ()){
            m_elephantFlowTable[it->first] = it->second;
          }
          elephant_num += 1;
        }
      }
    }
  }
}

void OFSwitch13TsfccController::RemoveFlowTable(const FlowStats &flow, const uint64_t out_dpid){
  Quadruple ack_key;
  ack_key.ipv4_src = flow.ipv4_dst;
  ack_key.ipv4_dst = flow.ipv4_src;
  ack_key.src_port = flow.dst_port;
  ack_key.dst_port = flow.src_port;
  auto ack_it = m_globalFlowTable.find(ack_key);
  if (ack_it != m_globalFlowTable.end ()){
    FlowStats ack_flow = ack_it->second;
    std::ostringstream remove_set_rwnd;
    remove_set_rwnd << "flow-mod cmd=del,table=0 eth_type=0x800,"
            << "ip_proto=6,ip_src=" 
            << ack_key.ipv4_src
            << ",ip_dst=" 
            << ack_key.ipv4_dst
            << ","
            << "tcp_src=" 
            << ack_key.src_port 
            << ",tcp_dst=" 
            << ack_key.dst_port;
    DpctlExecute (out_dpid, remove_set_rwnd.str());
  }
}
/**
 * @brief 找到这条流的第一跳交换机，并下发修改rwnd的流表（普通数据包）
 * 
 * @param flow TCP流（根据这条流找到ACK流）
 * @param rwnd 要修改的rwnd值
 */
void OFSwitch13TsfccController::SetRwnd(const FlowStats &flow, const uint16_t rwnd){
  Quadruple ack_key;
  ack_key.ipv4_src = flow.ipv4_dst;
  ack_key.ipv4_dst = flow.ipv4_src;
  ack_key.src_port = flow.dst_port;
  ack_key.dst_port = flow.src_port;
  //找到流第一跳交换机的输入端口，作为反向ack流的最后一跳输出端口，
  //利用流表找到这条流的相反方向流，修改rwnd
  uint64_t out_dpid = flow.switches.front().first;
  uint32_t out_port = flow.switches.front().second.in_port;
  std::ostringstream set_rwnd;
  set_rwnd << "flow-mod cmd=add,table=0,prio=720 eth_type=0x800,"
          << "ip_proto=6,ip_src=" 
          << ack_key.ipv4_src
          << ",ip_dst=" 
          << ack_key.ipv4_dst
          << ","
          << "tcp_src=" 
          << ack_key.src_port 
          << ",tcp_dst=" 
          << ack_key.dst_port 
          << " apply:set_rwnd=" 
          << rwnd 
          << ",output="
          << out_port;
  DpctlExecute (out_dpid, set_rwnd.str());
}

/**
 * @brief 找到这条流的第一跳交换机，并下发修改rwnd的流表（SYN-ACK数据包）
 * 
 * @param flow TCP流（根据这条流找到ACK流）
 * @param rwnd 要修改的rwnd值
 */
void OFSwitch13TsfccController::SetSYNRwnd(const FlowStats &flow, const uint16_t rwnd){
  Quadruple ack_key;
  ack_key.ipv4_src = flow.ipv4_dst;
  ack_key.ipv4_dst = flow.ipv4_src;
  ack_key.src_port = flow.dst_port;
  ack_key.dst_port = flow.src_port;
  //找到流第一跳交换机的输入端口，作为反向ack流的最后一跳输出端口，
  //利用流表找到这条流的相反方向流，修改rwnd
  uint64_t out_dpid = flow.switches.front().first;
  uint32_t out_port = flow.switches.front().second.in_port;
  std::ostringstream set_rwnd;
  set_rwnd << "flow-mod cmd=add,table=0,prio=840 eth_type=0x800,"
          << "ip_proto=6,tcp_flags=18,ip_src=" 
          << ack_key.ipv4_src
          << ",ip_dst=" 
          << ack_key.ipv4_dst
          << ","
          << "tcp_src=" 
          << ack_key.src_port 
          << ",tcp_dst=" 
          << ack_key.dst_port 
          << " apply:set_rwnd=" 
          << rwnd 
          << ",output="
          << out_port;
  DpctlExecute (out_dpid, set_rwnd.str());
}

/**
 * @brief 从全局流表中找到老鼠流调用SetRwnd或者SetSYNRwnd进行rwnd的修改
 * 
 * @param mou_rwnd 老鼠流修改的rwnd值
 * @param dpId 交换机ID
 * @param port_no 交换机出端口
 * @param flag 判断数据包是否是普通数据包（1：SYN-ACK数据包，0：普通数据包）
 */
void OFSwitch13TsfccController::UpdateMouseRWND(uint16_t mou_rwnd, uint64_t dpId, uint32_t port_no, bool flag){
  FlowTableMap_t::iterator it;
  NS_LOG_INFO("UpdateMouseRWND");
  //遍历全局流表
  for (it = m_globalFlowTable.begin(); it != m_globalFlowTable.end(); it++) {
    FlowStats flow = it->second;
    Quadruple key = it->first;
    std::vector<std::pair<uint64_t, SwitchInfo>>::iterator switch_it;
    //判断一条流所经过的交换机是否有当前拥塞的交换机
    for (switch_it = flow.switches.begin(); switch_it != flow.switches.end(); switch_it++) {
      if(dpId == switch_it->first && switch_it->second.out_port == port_no){
        auto it = m_elephantFlowTable.find(key);
        //如果不在大象流表中，那么认为这条流是老鼠流，用修改老鼠流的rwnd去修改
        if (it == m_elephantFlowTable.end ()){
          if(flag){
            SetSYNRwnd(flow, mou_rwnd);
          }else{
            SetRwnd(flow, mou_rwnd);
          }
        }
      }
    }
  }
}
/**
 * @brief 
 * 
 * @param ele_rwnd 从大象流表中找到大象流调用SetRwnd进行rwnd的修改
 * @param dpId 交换机ID
 * @param port_no 交换机出端口
 */
void OFSwitch13TsfccController::UpdateElephantRWND(uint16_t ele_rwnd, uint64_t dpId, uint32_t port_no){
  FlowTableMap_t::iterator it;
  NS_LOG_INFO("UpdateElephantRWND");
    //遍历大象流表
    for (it = m_elephantFlowTable.begin(); it != m_elephantFlowTable.end(); ++it) {
      FlowStats flow = it->second;
      // Quadruple key = it->first;
      std::vector<std::pair<uint64_t, SwitchInfo>>::iterator switch_it;
      //判断一条流所经过的交换机是否有当前拥塞的交换机
      for (switch_it = flow.switches.begin(); switch_it != flow.switches.end(); switch_it++) {
        if(dpId == switch_it->first && switch_it->second.out_port == port_no){
          SetRwnd(flow, ele_rwnd);
        }
      }
    }
}

/**
 * @brief 
 * 
 * @param ele_rwnd 从全局流表中找到所有流调用SetRwnd进行rwnd的修改
 * @param dpId 交换机ID
 * @param port_no 交换机出端口
 */
void OFSwitch13TsfccController::UpdateAllRWND(uint16_t rwnd, uint64_t dpId, uint32_t port_no){
  FlowTableMap_t::iterator it;
  NS_LOG_INFO("UpdateAllRWND");
  
  //遍历所有流表
  for (it = m_globalFlowTable.begin(); it != m_globalFlowTable.end(); ++it) {
    FlowStats flow = it->second;
    // Quadruple key = it->first;
    std::vector<std::pair<uint64_t, SwitchInfo>>::iterator switch_it;
    //判断一条流所经过的交换机是否有当前拥塞的交换机
    for (switch_it = flow.switches.begin(); switch_it != flow.switches.end(); switch_it++) {
      if(dpId == switch_it->first && switch_it->second.out_port == port_no){
        SetRwnd(flow, rwnd);
      }
    }
  }
}

/**
 * @brief 用于处理队列超过阈值接收到的OpenFlow消息，流程为：先区分象鼠流，再根据队列长度判断进行哪一种拥塞控制方案
 * 
 * @param msg OpenFlow消息
 * @param swtch 交换机
 * @param xid xid
 * @return ofl_err 错误结构体
 */
ofl_err
OFSwitch13TsfccController::HandleQueCn (
  struct ofl_msg_que_cn_cr *msg, Ptr<const RemoteSwitch> swtch,
  uint32_t xid)
{
  NS_LOG_FUNCTION (this << swtch << xid);
  uint16_t elephant_num = 0;
  uint16_t flow_num = 0;
  uint16_t ele_rwnd = 0;
  uint16_t mou_rwnd = 0;
  uint32_t port_no = msg->port_no;
  uint16_t queue_length = msg->queue_length;
  // Get the switch datapath ID
  uint64_t dpId = swtch->GetDpId ();
  
  //分类大象流老鼠流，并记录各自的流数量
  ClassifyTraffic(flow_num, elephant_num, dpId, port_no);
  
  if(flow_num != 0){
    //计算公平窗口
    double fair_window = (BDP + queue_length * 1500)/flow_num;
    // NS_LOG_WARN ("------" << elephant_num << "----------" << flow_num << "--------");

    //根据队列长度限制发送窗口
    if(elephant_num != 0 && queue_length != 0){
      if (queue_length < queue_threshold_h){
        double Alpha = (double)(2*flow_num - elephant_num + (queue_threshold_l * 1500)/2)/(elephant_num * fair_window);
        ele_rwnd =std::max(int((1-Alpha)*fair_window/4), int(max_size/4));
        // NS_LOG_WARN ("------" << Alpha << "----------" << ele_rwnd << "--------");
        UpdateElephantRWND(ele_rwnd, dpId, port_no);
      }else{
        ele_rwnd = int(max_size/4);

        //老鼠流rwnd设置
        mou_rwnd = std::max(int((BDP)/flow_num/4),int(max_size/4));
        //大象流就修改为1MSS
        UpdateElephantRWND(ele_rwnd, dpId, port_no);
        //老鼠流修改为mou_rwnd
        UpdateMouseRWND(mou_rwnd, dpId, port_no, false);
        
      }
    }
  }
  
  ofl_msg_free ((struct ofl_msg_header*)msg, 0);
  return 0;
}
/**
 * @brief 用于处理接收到队列长度恢复到阈值以下的OpenFlow消息，流程为：先区分象鼠流，再根据BDP等对象鼠流进行不同的rwnd值的增加
 * 
 * @param msg OpenFlow消息
 * @param swtch 交换机
 * @param xid xid
 * @return ofl_err 错误结构体
 */
ofl_err
OFSwitch13TsfccController::HandleQueCr (
  struct ofl_msg_que_cn_cr *msg, Ptr<const RemoteSwitch> swtch,
  uint32_t xid)
{
  NS_LOG_FUNCTION (this << swtch << xid);

  // Get the switch datapath ID
  uint64_t dpId = swtch->GetDpId ();
  uint16_t flow_num = 0;
  uint32_t port_no = msg->port_no;
  FlowTableMap_t::iterator it;
  for (it = m_globalFlowTable.begin(); it != m_globalFlowTable.end(); ++it) {
    FlowStats flow = it->second;
    std::vector<std::pair<uint64_t, SwitchInfo>>::iterator switch_it;
    for (switch_it = flow.switches.begin(); switch_it != flow.switches.end(); switch_it++) {
      if(dpId == switch_it->first && switch_it->second.out_port == port_no){
        flow_num += 1;
      }
    }
  }
  uint32_t rwnd = std::min(std::max(int((BDP + queue_threshold_l/2 * 1500)/flow_num/4),int(max_size/4)), int(131072/4));
  UpdateElephantRWND(rwnd, dpId, port_no);

  ofl_msg_free ((struct ofl_msg_header*)msg, 0);
  return 0;
}
/**
 * @brief 处理接收到的Sketch数据，包括10条流量的四元组，根据时间判断是否都是大象流
 * 
 * @param msg 接收到的OpenFlow消息
 * @param swtch 发送消息的交换机
 * @param xid xid
 * @return ofl_err 错误结构体
 */
ofl_err
OFSwitch13TsfccController::HandleSketchData (
  struct ofl_msg_sketch_data *msg, Ptr<const RemoteSwitch> swtch,
  uint32_t xid)
{
  NS_LOG_FUNCTION (this << swtch << xid);
  Quadruple flow_id;
  Quadruple ack_flow;
  FlowStats flow;
  for(int i = 0; i < 10; i++){
    if(ntohl(msg->elephant_flow[i].ip_src) != 0){
      flow_id.ipv4_src = Ipv4Address(ntohl(msg->elephant_flow[i].ip_src));
      flow_id.ipv4_dst = Ipv4Address(ntohl(msg->elephant_flow[i].ip_dst));
      flow_id.src_port = msg->elephant_flow[i].tcp_src;
      flow_id.dst_port = msg->elephant_flow[i].tcp_dst;
      // NS_LOG_WARN("ipv4_src: " << ntohl(msg->elephant_flow[i].ip_src) << " ipv4_dst: " << ntohl(msg->elephant_flow[i].ip_dst)
      //             << " src_port: " << (msg->elephant_flow[i].tcp_src) << " dst_port: " << (msg->elephant_flow[i].tcp_dst));
      auto it = m_globalFlowTable.find(flow_id);
      if (it != m_globalFlowTable.end ()){
        flow = it->second;
        Time now = Simulator::Now();
        flow.exist_time = (now - flow.start_time).GetSeconds();
        if(flow.exist_time > 0.2){
          auto eleIt = m_elephantFlowTable.find(it->first);
          if (eleIt == m_elephantFlowTable.end ()){
            m_elephantFlowTable[it->first] = it->second;
          }
          ack_flow.ipv4_src = flow_id.ipv4_dst;
          ack_flow.ipv4_dst = flow_id.ipv4_src;
          ack_flow.src_port = flow_id.dst_port;
          ack_flow.dst_port = flow_id.src_port;
          auto ack_it = m_globalFlowTable.find(ack_flow);
          if (ack_it != m_globalFlowTable.end ()){
            auto ack_eleIt = m_elephantFlowTable.find(ack_it->first);
            if (ack_eleIt == m_elephantFlowTable.end ()){
              m_elephantFlowTable[ack_it->first] = ack_it->second;
            }
          }
        }
      }
    }
  }
  ofl_msg_free ((struct ofl_msg_header*)msg, 0);
  return 0;
}
/**
 * @brief 用于处理经过交换机的SYN数据包，SYN数据包被认为是新流量，所以在全局流表添加新流以及预测Incast中流数量+1
 * 
 * @param ipv4_src 流的IP源地址
 * @param ipv4_dst 流的IP目的地址
 * @param src_port 流的TCP源端口
 * @param dst_port 流的TCP目的端口
 * @param inPort 交换机入端口
 * @param outPort 交换机出端口
 * @param dpid 交换机ID
 */
void OFSwitch13TsfccController::HandleSYN(const Ipv4Address &ipv4_src, const Ipv4Address &ipv4_dst, const uint16_t src_port, 
                const uint16_t dst_port, const uint32_t inPort, const uint32_t outPort, const uint64_t dpid)
{
  NS_LOG_DEBUG ("TCP FLAG IS: TCP_SYN");
  Quadruple flow_id;
  FlowStats flow;
  flow_id.ipv4_src = ipv4_src;
  flow_id.ipv4_dst = ipv4_dst;
  flow_id.src_port = src_port;
  flow_id.dst_port = dst_port;
  auto switchDataIterator = m_newFlowNumOnSwitchMap.find(dpid);
  if (switchDataIterator != m_newFlowNumOnSwitchMap.end()) {
      auto portDataIterator = switchDataIterator->second.find(outPort);
      if (portDataIterator != switchDataIterator->second.end()) {
        m_newFlowNumOnSwitchMap[dpid][outPort] += 1;
      } else {
        m_newFlowNumOnSwitchMap[dpid][outPort] = 1;
      }
  } else {
      m_newFlowNumOnSwitchMap[dpid][outPort] = 1;
  }
  auto it = m_globalFlowTable.find(flow_id);
  if (it == m_globalFlowTable.end ()){
    flow.ipv4_src = ipv4_src;
    flow.ipv4_dst = ipv4_dst;
    flow.src_port = src_port;
    flow.dst_port = dst_port;
    flow.max_size = 1460;
    flow.shift_cnt = 0;
    flow.size = 0;
    flow.start_time = Simulator::Now();
    flow.exist_time = 0.0;
    //将第一个交换机的输入和输出端口插入流表中
    flow.switches = std::vector<std::pair<uint64_t, SwitchInfo>>();
    SwitchInfo portInfo;
    portInfo.in_port = inPort;
    portInfo.out_port = outPort;
    std::pair<uint64_t, SwitchInfo> switchEntry (dpid, portInfo);
    flow.switches.push_back(switchEntry);
    
    //将新的流量插入流表中
    std::pair<Quadruple, FlowStats> flowEntry (flow_id, flow);
    auto retGFT = m_globalFlowTable.insert (flowEntry);
    if (retGFT.second == false)
    {
      NS_LOG_ERROR ("Can't insert m_globalFlowTable pair");
    }
  }else{
    //流量已存在流表，只更新交换机列表
    bool flag = true;
    flow = it->second;
    for (const auto& entry : flow.switches) {
      if (entry.first == dpid) {
        flag = false;
        break;
      }
    }
    if(flag == true){
      SwitchInfo portInfo;
      portInfo.in_port = inPort;
      portInfo.out_port = outPort;
      std::pair<uint64_t, SwitchInfo> switchEntry (dpid, portInfo);
      flow.switches.push_back(switchEntry);
    }
  }
}
/**
  * @brief 用于处理经过交换机的FIN数据包，FIN数据包被认为是流量的结束，所以在全局流表删除新流以及删除这条流在所有交换机上的流表项
 * 
 * @param ipv4_src 流的IP源地址
 * @param ipv4_dst 流的IP目的地址
 * @param src_port 流的TCP源端口
 * @param dst_port 流的TCP目的端口
 */
void OFSwitch13TsfccController::HandleFIN(const Ipv4Address &ipv4_src, const Ipv4Address &ipv4_dst, 
                                          const uint16_t src_port, const uint16_t dst_port)
{
  //从数据结构中删除流表
  // NS_LOG_DEBUG ("TCP FLAG IS: TCP_FIN");
  Quadruple key;
  key.ipv4_src = ipv4_src;
  key.ipv4_dst = ipv4_dst;
  key.src_port = src_port;
  key.dst_port = dst_port;
  auto it = m_globalFlowTable.find(key);
  if (it != m_globalFlowTable.end ()){
    //下发删除交换机的修改这条流窗口的流表
    FlowStats flow = it->second;
    uint64_t out_dpid = flow.switches.front().first;
    Quadruple ack_key;
    ack_key.ipv4_src = ipv4_dst;
    ack_key.ipv4_dst = ipv4_src;
    ack_key.src_port = dst_port;
    ack_key.dst_port = src_port;
    auto ack_it = m_globalFlowTable.find(ack_key);
    if (ack_it != m_globalFlowTable.end ()){
      FlowStats ack_flow = ack_it->second;
      std::ostringstream remove_set_rwnd;
      remove_set_rwnd << "flow-mod cmd=del,table=0 eth_type=0x800,"
              << "ip_proto=6,ip_src=" 
              << ack_key.ipv4_src
              << ",ip_dst=" 
              << ack_key.ipv4_dst
              << ","
              << "tcp_src=" 
              << ack_key.src_port 
              << ",tcp_dst=" 
              << ack_key.dst_port;
      DpctlExecute (out_dpid, remove_set_rwnd.str());
    }

    auto ele_it = m_elephantFlowTable.find(key);
    if (ele_it != m_elephantFlowTable.end ()){
      m_elephantFlowTable.erase(ele_it);
    }
    m_globalFlowTable.erase(it);
  }
}
/**
 * @brief 处理交换机TABLE_MISS的数据包
 * 
 * @param msg OpenFlow消息
 * @param swtch 交换机
 * @param xid xid
 * @return ofl_err 错误结构体
 */
ofl_err
OFSwitch13TsfccController::HandlePacketIn (
  struct ofl_msg_packet_in *msg, Ptr<const RemoteSwitch> swtch,
  uint32_t xid)
{
  NS_LOG_FUNCTION (this << swtch << xid);

  static int prio = 20;
  uint32_t outPort = OFPP_FLOOD;
  enum ofp_packet_in_reason reason = msg->reason;

  // Get the switch datapath ID
  uint64_t dpid = swtch->GetDpId ();

  char *msgStr =
    ofl_structs_match_to_string ((struct ofl_match_header*)msg->match, 0);
  // NS_LOG_DEBUG ("Packet in match: " << msgStr);
  free (msgStr);

  if (reason == OFPR_NO_MATCH || reason == OFPR_ACTION)
    {
      // Let's get necessary information (input port and mac address)
      uint32_t inPort;
      size_t portLen = OXM_LENGTH (OXM_OF_IN_PORT); // (Always 4 bytes)
      struct ofl_match_tlv *input =
        oxm_match_lookup (OXM_OF_IN_PORT, (struct ofl_match*)msg->match);
      memcpy (&inPort, input->value, portLen);
      // NS_LOG_DEBUG ("------------" << inPort);
      Mac48Address src48;
      struct ofl_match_tlv *ethSrc =
        oxm_match_lookup (OXM_OF_ETH_SRC, (struct ofl_match*)msg->match);
      src48.CopyFrom (ethSrc->value);

      Mac48Address dst48;
      struct ofl_match_tlv *ethDst =
        oxm_match_lookup (OXM_OF_ETH_DST, (struct ofl_match*)msg->match);
      dst48.CopyFrom (ethDst->value);

      // Get L2Table for this datapath
      auto it = m_learnedInfo.find (dpid);
      if (it != m_learnedInfo.end ())
        {
          L2Table_t *l2Table = &it->second;

          // Looking for out port based on dst address (except for broadcast)
          if (!dst48.IsBroadcast ())
            {
              auto itDst = l2Table->find (dst48);
              if (itDst != l2Table->end ())
                {
                  outPort = itDst->second;
                }
              else
                {
                  NS_LOG_DEBUG ("No L2 info for mac " << dst48 << ". Flood.");
                }
            }

          // Learning port from source address
          NS_ASSERT_MSG (!src48.IsBroadcast (), "Invalid src broadcast addr");
          auto itSrc = l2Table->find (src48);
          if (itSrc == l2Table->end ())
            {
              std::pair<Mac48Address, uint32_t> entry (src48, inPort);
              auto ret = l2Table->insert (entry);
              if (ret.second == false)
                {
                  NS_LOG_ERROR ("Can't insert mac48address / port pair");
                }
              else
                {
                  NS_LOG_DEBUG ("Learning that mac " << src48 <<
                                " can be found at port " << inPort);

                  // Send a flow-mod to switch creating this flow. Let's
                  // configure the flow entry to 10s idle timeout and to
                  // notify the controller when flow expires. (flags=0x0001)
                  std::ostringstream cmd;
                  cmd << "flow-mod cmd=add,table=0,idle=10,flags=0x0001"
                      << ",prio=" << prio << " eth_dst=" << src48
                      << " apply:output=" << inPort;
                  DpctlExecute (dpid, cmd.str ());
                }
            }
          else
            {
              NS_ASSERT_MSG (itSrc->second == inPort,
                             "Inconsistent L2 switching table");
            }
        }
      else
        {
          NS_LOG_ERROR ("No L2 table for this datapath id " << dpid);
        }

      uint8_t isTCP;
      struct ofl_match_tlv *ip_proto =
        oxm_match_lookup (OXM_OF_IP_PROTO, (struct ofl_match*)msg->match);
      if(ip_proto != NULL){
        memcpy(&isTCP, ip_proto->value, OXM_LENGTH(OXM_OF_IP_PROTO));
        if(isTCP == 6){
          Ipv4Address ipv4_src;
          struct ofl_match_tlv *ipv4Src =
            oxm_match_lookup (OXM_OF_IPV4_SRC, (struct ofl_match*)msg->match);
          // memcpy(&ipv4_src, ipv4Src->value, OXM_LENGTH(OXM_OF_IPV4_SRC));
          uint32_t ipv4SrcValue = ntohl(*(uint32_t*)ipv4Src->value);
          ipv4_src = Ipv4Address(ipv4SrcValue);

          Ipv4Address ipv4_dst;
          struct ofl_match_tlv *ipv4Dst =
            oxm_match_lookup (OXM_OF_IPV4_DST, (struct ofl_match*)msg->match);
          // memcpy(&ipv4_dst, ipv4Dst->value, OXM_LENGTH(OXM_OF_IPV4_DST));
          uint32_t ipv4DstValue = ntohl(*(uint32_t*)ipv4Dst->value);
          ipv4_dst = Ipv4Address(ipv4DstValue);

          struct ofl_match_tlv* tlv;
          uint16_t src_port;
          uint16_t dst_port;
          tlv = oxm_match_lookup(OXM_OF_TCP_SRC, (struct ofl_match*)msg->match);
          memcpy(&src_port, tlv->value, OXM_LENGTH(OXM_OF_TCP_SRC));
          tlv = oxm_match_lookup(OXM_OF_TCP_DST, (struct ofl_match*)msg->match);
          memcpy(&dst_port, tlv->value, OXM_LENGTH(OXM_OF_TCP_DST));
          int tcpFlags;
          tlv = oxm_match_lookup(OXM_OF_TCP_FLAGS, (struct ofl_match*)msg->match);
          memcpy(&tcpFlags, tlv->value, OXM_LENGTH(OXM_OF_TCP_FLAGS));

          if (tcpFlags & TCP_SYN) {
            HandleSYN(ipv4_src, ipv4_dst, src_port, dst_port, inPort, outPort, dpid);
          }
          if (tcpFlags & TCP_FIN) {
            HandleFIN(ipv4_src, ipv4_dst, src_port, dst_port);
          }

        }
      }
      // Lets send the packet out to switch.
      struct ofl_msg_packet_out reply;
      reply.header.type = OFPT_PACKET_OUT;
      reply.buffer_id = msg->buffer_id;
      reply.in_port = inPort;
      reply.data_length = 0;
      reply.data = 0;

      if (msg->buffer_id == NO_BUFFER)
        {
          // No packet buffer. Send data back to switch
          reply.data_length = msg->data_length;
          reply.data = msg->data;
        }

      // Create output action
      struct ofl_action_output *a =
        (struct ofl_action_output*)xmalloc (sizeof (struct ofl_action_output));
      a->header.type = OFPAT_OUTPUT;
      a->port = outPort;
      a->max_len = 0;

      reply.actions_num = 1;
      reply.actions = (struct ofl_action_header**)&a;


      SendToSwitch (swtch, (struct ofl_msg_header*)&reply, xid);
      free (a);
    }
  else
    {
      NS_LOG_WARN ("This controller can't handle the packet. Unkwnon reason.");
    }

  // All handlers must free the message when everything is ok
  ofl_msg_free ((struct ofl_msg_header*)msg, 0);
  return 0;
}

ofl_err
OFSwitch13TsfccController::HandleFlowRemoved (
  struct ofl_msg_flow_removed *msg, Ptr<const RemoteSwitch> swtch,
  uint32_t xid)
{
  NS_LOG_FUNCTION (this << swtch << xid);

  // Get the switch datapath ID
  uint64_t dpid = swtch->GetDpId ();

  NS_LOG_DEBUG ( "Flow entry expired. Removing from L2 switch table.");
  auto it = m_learnedInfo.find (dpid);
  if (it != m_learnedInfo.end ())
    {
      Mac48Address mac48;
      struct ofl_match_tlv *ethSrc =
        oxm_match_lookup (OXM_OF_ETH_DST, (struct ofl_match*)msg->stats->match);
      mac48.CopyFrom (ethSrc->value);

      L2Table_t *l2Table = &it->second;
      auto itSrc = l2Table->find (mac48);
      if (itSrc != l2Table->end ())
        {
          l2Table->erase (itSrc);
        }
    }

  // All handlers must free the message when everything is ok
  ofl_msg_free_flow_removed (msg, true, 0);
  return 0;
}

/**
 * @brief 握手成功，交换机和控制器成功连接，下发一些必要的流表
 * 
 * @param swtch 交换机
 */
void
OFSwitch13TsfccController::HandshakeSuccessful (
  Ptr<const RemoteSwitch> swtch)
{
  NS_LOG_FUNCTION (this << swtch);

  // Get the switch datapath ID
  uint64_t dpid = swtch->GetDpId ();

  // After a successfull handshake, let's install the table-miss entry, setting
  // to 128 bytes the maximum amount of data from a packet that should be sent
  // to the controller.
  DpctlExecute (dpid, "flow-mod cmd=add,table=0,prio=0 "
                "apply:output=ctrl:128");
  DpctlExecute (dpid, "flow-mod cmd=add,table=0,prio=600 eth_type=0x800,ip_proto=6,tcp_flags=2 apply:output=ctrl:128");
  DpctlExecute (dpid, "flow-mod cmd=add,table=0,prio=600 eth_type=0x800,ip_proto=6,tcp_flags=17 apply:output=ctrl:128");
  DpctlExecute (dpid, "flow-mod cmd=add,table=0,prio=600 eth_type=0x800,ip_proto=6,tcp_flags=18 apply:output=ctrl:128");
  // std::string flowTable = "stats-flow table=0";
  // DpctlExecute (dpid, flowTable);
  // Configure te switch to buffer packets and send only the first 128 bytes of
  // each packet sent to the controller when not using an output action to the
  // OFPP_CONTROLLER logical port.
  DpctlExecute (dpid, "set-config miss=128");
  Simulator::Schedule (Seconds (0.00006), &OFSwitch13TsfccController::PredictIncast, this);
  // Create an empty L2SwitchingTable and insert it into m_learnedInfo
  L2Table_t l2Table;
  std::pair<uint64_t, L2Table_t> entry (dpid, l2Table);
  auto ret = m_learnedInfo.insert (entry);
  if (ret.second == false)
    {
      NS_LOG_ERROR ("Table exists for this datapath.");
    }
}

} // namespace ns3
#endif // NS3_OFSWITCH13
