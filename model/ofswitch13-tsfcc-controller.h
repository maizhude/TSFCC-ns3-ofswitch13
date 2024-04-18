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

#ifndef OFSWITCH13_TSFCC_CONTROLLER_H
#define OFSWITCH13_TSFCC_CONTROLLER_H

#include "ofswitch13-controller.h"
#include <ctime>
#define TCP_FIN 0x01
#define TCP_SYN 0x02
#define TCP_RST 0x04
#define TCP_PSH 0x08
#define TCP_ACK 0x10
#define TCP_URG 0x20
namespace ns3 {

/**
 * \ingroup ofswitch13
 * \brief An Learning OpenFlow 1.3 controller (works as L2 switch)
 */
class OFSwitch13TsfccController : public OFSwitch13Controller
{
protected:
  struct FlowStats;
public:
  OFSwitch13TsfccController ();          //!< Default constructor
  virtual ~OFSwitch13TsfccController (); //!< Dummy destructor.

  /**
   * Register this type.
   * \return The object TypeId.
   */
  static TypeId GetTypeId (void);

  /** Destructor implementation */
  virtual void DoDispose ();

  /**
   * Handle packet-in messages sent from switch to this controller. Look for L2
   * switching information, update the structures and send a packet-out back.
   *
   * \param msg The packet-in message.
   * \param swtch The switch information.
   * \param xid Transaction id.
   * \return 0 if everything's ok, otherwise an error number.
   */
  ofl_err HandlePacketIn (
    struct ofl_msg_packet_in *msg, Ptr<const RemoteSwitch> swtch,
    uint32_t xid);

  ofl_err HandleQueCn (
    struct ofl_msg_que_cn_cr *msg, Ptr<const RemoteSwitch> swtch,
    uint32_t xid);

  ofl_err HandleQueCr (
    struct ofl_msg_que_cn_cr *msg, Ptr<const RemoteSwitch> swtch,
    uint32_t xid);
  
  ofl_err HandleSketchData (
  struct ofl_msg_sketch_data *msg, Ptr<const RemoteSwitch> swtch,
  uint32_t xid);

  void ClassifyTraffic(uint16_t &flow_num, uint16_t &elephant_num, uint64_t dpId, uint32_t port_no);

  void RemoveFlowTable(const FlowStats &flow, const uint64_t out_dpid);

  void PredictIncast();

  void SetRwnd(const FlowStats &flow, const uint16_t rwnd);

  void SetSYNRwnd(const FlowStats &flow, const uint16_t rwnd);

  void UpdateMouseRWND(uint16_t mou_rwnd, uint64_t dpId, uint32_t port_no, bool flag);

  void UpdateElephantRWND(uint16_t ele_rwnd, uint64_t dpId, uint32_t port_no);

   void UpdateAllRWND(uint16_t rwnd, uint64_t dpId, uint32_t port_no);

  void HandleSYN(const Ipv4Address &ipv4_src, const Ipv4Address &ipv4_dst, const uint16_t src_port, 
                const uint16_t dst_port, const uint32_t inPort, const uint32_t outPort, const uint64_t dpid);

  void HandleFIN(const Ipv4Address &ipv4_src, const Ipv4Address &ipv4_dst, const uint16_t src_port, const uint16_t dst_port);
  /**
   * Handle flow removed messages sent from switch to this controller. Look for
   * L2 switching information and removes associated entry.
   *
   * \param msg The flow removed message.
   * \param swtch The switch information.
   * \param xid Transaction id.
   * \return 0 if everything's ok, otherwise an error number.
   */
  ofl_err HandleFlowRemoved (
    struct ofl_msg_flow_removed *msg, Ptr<const RemoteSwitch> swtch,
    uint32_t xid);

protected:
  // Inherited from OFSwitch13Controller
  void HandshakeSuccessful (Ptr<const RemoteSwitch> swtch);

  struct SwitchInfo
  {
    friend class OFSwitch13TsfccController;
private:
    uint32_t in_port;
    uint32_t out_port;
  };

  struct Quadruple
  {
    friend class OFSwitch13TsfccController;
private:
    Ipv4Address ipv4_src;
    Ipv4Address ipv4_dst;
    uint16_t src_port;
    uint16_t dst_port;
public:
    // 重载 == 操作符
    bool operator==(const Quadruple& other) const {
      uint32_t thisSrcIp = ipv4_src.Get();
      uint32_t otherSrcIp = other.ipv4_src.Get();
      uint32_t thisDstIp = ipv4_src.Get();
      uint32_t otherDstIp = other.ipv4_src.Get();
      return (thisSrcIp == otherSrcIp) &&
              (thisDstIp == otherDstIp) &&
              (src_port == other.src_port) &&
              (dst_port == other.dst_port);
    }
    bool operator<(const Quadruple& other) const {
      // 在这里定义自定义的大小比较规则
      // 返回 true 表示当前对象小于 other 对象
      // 返回 false 表示当前对象大于等于 other 对象
      // 你可以根据 ipv4_src、ipv4_dst、src_port、dst_port 等成员来进行比较
      // 这里只是一个示例，具体的比较规则根据你的需求来定义
      uint32_t thisSrcIp = ipv4_src.Get();
      uint32_t otherSrcIp = other.ipv4_src.Get();
      uint32_t thisDstIp = ipv4_src.Get();
      uint32_t otherDstIp = other.ipv4_src.Get();
      if (thisSrcIp < otherSrcIp) {return true;}
      else if (thisSrcIp > otherSrcIp) {return false;}
      else {
        if (thisDstIp < otherDstIp) {return true;}
        else if (thisDstIp > otherDstIp) {return false;}
        else{
          if (src_port < other.src_port) {return true;}
          else if (src_port > other.src_port) {return false;}
          else{
            if (dst_port < other.dst_port) {return true;}
            else if (dst_port > other.dst_port) {return false;}
          }
        }
      }
      // 继续比较其他成员...
      return false;
    }
  };
  struct FlowStats
  {
    friend class OFSwitch13TsfccController;
private:
    Ipv4Address ipv4_src;
    Ipv4Address ipv4_dst;
    uint16_t src_port;
    uint16_t dst_port;
    uint16_t max_size;
    uint16_t shift_cnt;
    uint16_t size;
    Time start_time;
    double exist_time;
    std::vector<std::pair<uint64_t, SwitchInfo>> switches;
  };

private:
  /** Map saving <IPv4 address / MAC address> */
  typedef std::map<Ipv4Address, Mac48Address> IpMacMap_t;
  IpMacMap_t m_arpTable; //!< ARP resolution table.

  /**
   * \name L2 switching structures
   */
  //\{
  /** L2SwitchingTable: map MacAddress to port */
  typedef std::map<Mac48Address, uint32_t> L2Table_t;

  /** Map datapathID to L2SwitchingTable */
  typedef std::map<uint64_t, L2Table_t> DatapathMap_t;

  typedef std::map<Quadruple,FlowStats> FlowTableMap_t;

  typedef std::map<uint32_t, uint32_t> NewFLowNumOnPortMap_t;

  typedef std::map<uint64_t, NewFLowNumOnPortMap_t> NewFlowNumOnSwitchMap_t;
  /** Switching information for all dapataths */
  DatapathMap_t m_learnedInfo;
  FlowTableMap_t m_globalFlowTable;
  FlowTableMap_t m_elephantFlowTable;
  NewFlowNumOnSwitchMap_t m_newFlowNumOnSwitchMap;
  //\}
  uint16_t max_size = 1460;
  double bandwith = 10.0 * 1024 * 1024 * 1024;
  uint16_t rtt = 600; //RTT为40us
  uint16_t queue_threshold_h = 200;
  uint16_t queue_threshold_l = 60;
  uint32_t IncastThreshold = 70;
  double BDP = (bandwith * rtt/1000000)/8;

};

} // namespace ns3
#endif /* OFSWITCH13_LEARNING_CONTROLLER_H */
