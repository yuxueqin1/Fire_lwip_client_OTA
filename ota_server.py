#!/usr/bin/env python3
"""
OTA服务器参考实现 - 服务器推送升级模式
用于测试STM32固件远程更新功能
服务器主动推送升级请求，客户端确认后开始下载
"""

import socket
import struct
import os
import sys
import argparse
import threading
import time
from zlib import crc32

# OTA命令定义
OTA_CMD_HELLO = 0x00
OTA_CMD_HELLO_RESP = 0x80
OTA_CMD_PUSH_UPDATE = 0x10
OTA_CMD_CONFIRM_UPDATE = 0x11
OTA_CMD_REJECT_UPDATE = 0x12
OTA_CMD_ERROR_UPDATE = 0x13
OTA_CMD_START_DOWNLOAD = 0x03
OTA_CMD_DATA = 0x04
OTA_CMD_END = 0x05
OTA_CMD_ACK = 0x06
OTA_CMD_NAK = 0x07

# 帧标记
OTA_NET_FRAME_MAGIC = 0xAA
OTA_NET_FRAME_END = 0xBB

class OTAFrame:
    def __init__(self, cmd, seq, data=b''):
        self.cmd = cmd
        self.seq = seq
        self.data = data
        self.offset = 0
        self.len = len(data)
    
    def to_bytes(self):
        """将帧编码为字节"""
        buffer = bytearray()
        buffer.append(OTA_NET_FRAME_MAGIC)
        buffer.append(self.cmd)
        buffer.append(self.seq)
        buffer.append((self.len >> 8) & 0xFF)
        buffer.append(self.len & 0xFF)
        buffer.append((self.offset >> 24) & 0xFF)
        buffer.append((self.offset >> 16) & 0xFF)
        buffer.append((self.offset >> 8) & 0xFF)
        buffer.append(self.offset & 0xFF)
        buffer.extend(self.data)
        
        # 计算校验和
        checksum = sum(buffer) & 0xFF
        buffer.append(checksum)
        buffer.append(OTA_NET_FRAME_END)
        
        return bytes(buffer)
    
    @staticmethod
    def from_bytes(data):
        """从字节解码帧"""
        if len(data) < 11:
            return None
        
        if data[0] != OTA_NET_FRAME_MAGIC or data[-1] != OTA_NET_FRAME_END:
            return None
        
        cmd = data[1]
        seq = data[2]
        length = (data[3] << 8) | data[4]
        offset = (data[5] << 24) | (data[6] << 16) | (data[7] << 8) | data[8]
        
        payload = data[9:9+length]
        checksum = data[9+length]
        
        # 验证校验和
        calc_sum = sum(data[:-2]) & 0xFF
        if calc_sum != checksum:
            print(f"[ERROR] Checksum mismatch: {calc_sum} != {checksum}")
            return None
        
        frame = OTAFrame(cmd, seq, payload)
        frame.offset = offset
        return frame


class OTAServer:
    def __init__(self, firmware_file, version, bind_addr='0.0.0.0', port=5002):
        self.firmware_file = firmware_file
        self.version = version
        self.bind_addr = bind_addr
        self.port = port
        self.firmware_data = None
        self.firmware_crc = 0
        self.firmware_size = 0
        self.clients = {}  # 连接的客户端列表
        self.seq = 0
        
        # 加载固件文件
        self.load_firmware()
    
    def load_firmware(self):
        """加载固件文件"""
        if not os.path.exists(self.firmware_file):
            print(f"[ERROR] Firmware file not found: {self.firmware_file}")
            sys.exit(1)
        
        with open(self.firmware_file, 'rb') as f:
            self.firmware_data = f.read()
        
        self.firmware_size = len(self.firmware_data)
        # 计算CRC32（与STM32兼容）
        self.firmware_crc = crc32(self.firmware_data) & 0xFFFFFFFF
        
        print(f"[INFO] Firmware loaded: {self.firmware_file}")
        print(f"[INFO] Size: {self.firmware_size} bytes")
        print(f"[INFO] CRC32: 0x{self.firmware_crc:08X}")
        print(f"[INFO] Version: 0x{self.version:08X}")
    
    def handle_client(self, client_sock, addr):
        """处理客户端连接"""
        print(f"\n[INFO] Client connected: {addr[0]}:{addr[1]}")
        
        client_id = f"{addr[0]}:{addr[1]}"
        self.clients[client_id] = {'sock': client_sock, 'addr': addr, 'version': 0, 'state': 'HELLO'}
        
        try:
            seq = 0
            need_update = False
            
            while True:
                # 接收客户端消息
                recv_data = client_sock.recv(2048)
                if not recv_data:
                    print(f"[INFO] Client {client_id} disconnected")
                    break
                
                frame = OTAFrame.from_bytes(recv_data)
                if not frame:
                    print(f"[ERROR] Failed to decode frame from {client_id}")
                    continue
                
                print(f"[RX from {client_id}] Command: 0x{frame.cmd:02X}, Seq: {frame.seq}")
                
                if frame.cmd == OTA_CMD_HELLO:
                    # 客户端报告当前版本
                    if len(frame.data) >= 4:
                        client_version = struct.unpack('>I', frame.data[:4])[0]
                        self.clients[client_id]['version'] = client_version
                        print(f"[INFO] Client {client_id} version: 0x{client_version:08X}")
                    
                    # 记录序列号
                    seq = frame.seq
                    self.clients[client_id]['state'] = 'HELLO'
                    
                    # 如果版本更低，主动推送升级请求
                    if client_version < self.version:
                        print(f"[INFO] New version available for {client_id}, pushing update request...")
                        
                        # 构建推送更新请求
                        resp_data = struct.pack('>I', self.version) + \
                                   struct.pack('>I', self.firmware_size) + \
                                   struct.pack('>I', self.firmware_crc)
                        resp = OTAFrame(OTA_CMD_PUSH_UPDATE, seq + 1, resp_data)
                        
                        client_sock.sendall(resp.to_bytes())
                        print(f"[TX to {client_id}] PUSH_UPDATE: v0x{self.version:08X}, size={self.firmware_size}")
                        self.clients[client_id]['state'] = 'WAITING_CONFIRM'
                        need_update = True
                    else:
                        print(f"[INFO] Client {client_id} already has latest version")
                        need_update = False
                
                elif frame.cmd == OTA_CMD_CONFIRM_UPDATE:
                    # 客户端同意升级
                    print(f"[INFO] Client {client_id} confirmed update")
                    self.clients[client_id]['state'] = 'DOWNLOADING'
                    need_update = True
                    seq = frame.seq
                    
                    # 开始发送固件
                    print(f"[INFO] Starting firmware transmission to {client_id}")
                    
                    # 发送START_DOWNLOAD
                    start_frame = OTAFrame(OTA_CMD_START_DOWNLOAD, seq + 1)
                    try:
                        client_sock.sendall(start_frame.to_bytes())
                        print(f"[TX to {client_id}] START_DOWNLOAD")
                    except socket.error as sock_err:
                        print(f"[ERROR] Socket error while sending START_DOWNLOAD: {sock_err}")
                        break
                    
                    # 分块发送固件
                    chunk_size = 128  # 减小块大小以提高可靠性
                    offset = 0
                    chunk_seq = seq + 2
                    transmission_error = False
                    
                    while offset < self.firmware_size:
                        chunk_len = min(chunk_size, self.firmware_size - offset)
                        chunk = self.firmware_data[offset:offset + chunk_len]
                        
                        data_frame = OTAFrame(OTA_CMD_DATA, chunk_seq & 0xFF, chunk)
                        data_frame.offset = offset
                        
                        try:
                            client_sock.sendall(data_frame.to_bytes())
                            time.sleep(1)
                        except socket.error as sock_err:
                            print(f"[ERROR] Socket error while sending chunk at offset {offset}: {sock_err}")
                            transmission_error = True
                            break
                        
                        offset += chunk_len
                        progress = (offset * 100) // self.firmware_size
                        
                        if progress % 10 == 0:
                            print(f"[INFO] Sending to {client_id}: {offset}/{self.firmware_size} bytes ({progress}%)")
                        
                        chunk_seq += 1
                        # 关键：增加延迟时间让客户端有足够时间写入Flash
                        # STM32写512字节需要128-256ms (每次HAL_FLASH_Program约1-2ms, 需要128次)
                        time.sleep(1)  # 300ms延迟让客户端完成Flash写入
                    
                    if not transmission_error:
                        # 发送END帧
                        end_frame = OTAFrame(OTA_CMD_END, chunk_seq & 0xFF)
                        try:
                            client_sock.sendall(end_frame.to_bytes())
                            print(f"[TX to {client_id}] END - Firmware transmission completed")
                            self.clients[client_id]['state'] = 'COMPLETED'
                        except socket.error as sock_err:
                            print(f"[ERROR] Socket error while sending END frame: {sock_err}")
                    
                    # 固件发送完成或出错，关闭连接
                    break
                
                elif frame.cmd == OTA_CMD_REJECT_UPDATE:
                    # 客户端拒绝升级
                    print(f"[INFO] Client {client_id} rejected update")
                    self.clients[client_id]['state'] = 'IDLE'
                    need_update = False

                elif frame.cmd == OTA_CMD_ERROR_UPDATE:
                    # 客户端报告错误
                    if len(frame.data) >= 4:
                        error_code = struct.unpack('>I', frame.data[:4])[0]
                        print(f"[ERROR] Client {client_id} reported error: 0x{error_code:08X}")
                    self.clients[client_id]['state'] = 'ERROR'
                    need_update = False
        
        except Exception as e:
            print(f"[ERROR] Client handler error for {client_id}: {e}")
        
        finally:
            if client_id in self.clients:
                del self.clients[client_id]
            client_sock.close()
            print(f"[INFO] Client {client_id} handler stopped")
    
    def run(self):
        """运行OTA服务器"""
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            server_sock.bind((self.bind_addr, self.port))
            server_sock.listen(5)
            
            print(f"[INFO] OTA Server listening on {self.bind_addr}:{self.port}")
            print(f"[INFO] Mode: Server-push (客户端被动接收升级请求)")
            print("[INFO] Waiting for client connections...")
            
            while True:
                try:
                    client_sock, addr = server_sock.accept()
                    # 为每个客户端创建独立的处理线程
                    client_thread = threading.Thread(
                        target=self.handle_client,
                        args=(client_sock, addr),
                        daemon=True
                    )
                    client_thread.start()
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    print(f"[ERROR] Connection error: {e}")
        
        except Exception as e:
            print(f"[ERROR] Server error: {e}")
        
        finally:
            server_sock.close()
            print("[INFO] OTA Server stopped")


def main():
    parser = argparse.ArgumentParser(description='OTA Server for STM32 firmware updates (Server-Push Mode)')
    parser.add_argument('-f', '--firmware', required=True, help='Firmware file path')
    parser.add_argument('-v', '--version', type=lambda x: int(x, 0), default=0x01000001, 
                       help='Firmware version (hex, default: 0x01000001)')
    parser.add_argument('-p', '--port', type=int, default=5002, help='Server port (default: 5002)')
    parser.add_argument('-b', '--bind', default='0.0.0.0', help='Bind address (default: 0.0.0.0)')
    
    args = parser.parse_args()
    
    server = OTAServer(args.firmware, args.version, args.bind, args.port)
    server.run()


if __name__ == '__main__':
    main()

