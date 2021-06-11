#pragma once
namespace bz_robot
{
/*
 * 名称：             uart_open
 * 功能：             打开串口并返回串口设备文件描述
 * 入口参数：          fd      文件描述符
                     port    串口号(ttyS0,ttyS1,ttyS2, ttyUSB0)
 * 出口参数：          错误返回为-1
 */
int uart_open(int fd, const char *port);
/*
 * 名称：             uart_close
 * 功能：             关闭串口并返回串口设备文件描述
 * 入口参数：         fd          文件描述符
                    port        串口号(ttyS0,ttyS1,ttyS2)
 * 出口参数：void
 */

void uart_close(int fd);
/*
 *名称：             uart_set
 *功能：             设置串口数据位，停止位和效验位
 *入口参数：         fd          串口文件描述符
 *                   speed       串口速度
 *                   flow_ctrl   数据流控制
 *                   databits    数据位   取值为 7 或者8
 *                   stopbits    停止位   取值为 1 或者2
 *                   parity      效验类型 取值为N,E,O,,S
 *出口参数：正确返回为1，错误返回为0
 */
bool uart_set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity);
/*******************************************************************
*名称：                uart_init()
*功能：                串口初始化
*入口参数：            fd         文件描述符
*                      speed      串口速度
*                      flow_ctrl  数据流控制
*                      databits   数据位   取值为 7 或者8
*                      stopbits   停止位   取值为 1 或者2
*                      parity     效验类型 取值为N,E,O,,S
*
*出口参数：正确返回为1，错误返回为0
*******************************************************************/
bool uart_init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity);
/*
 * 名称：            uart_recv
 * 功能：            接收串口数据
 * 入口参数：         fd         文件描述符
 *                  rcv_buf    接收串口中数据存入rcv_buf缓冲区中
 *                  data_len   一帧数据的长度
 * 出口参数：        正确返回为接收到的数据长度，错误返回为0
 */
int uart_recv(int fd, char *rcv_buf,int data_len);
/*
 * 名称：            uart_send
 * 功能：            发送数据
 * 入口参数：        fd           文件描述符
 *                   send_buf     存放串口发送数据
 *                   data_len     一帧数据的个数
 * 出口参数：        正确返回为1，错误返回为-1
 */
int uart_send(int fd, const char *send_buf, int data_len);
}
