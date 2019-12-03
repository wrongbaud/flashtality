import socket
import sys
import argparse

def esp_connect(ip,port):
    try:
        s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        s.connect((ip,port))
    except socket.error as msg:
        print msg
        sys.exit()
    return s

def dump_flash(sock,start_addr,size,ofile):
    cmd_str = "r:{:X}:{:X}".format(start_addr,size)
    sock.send(cmd_str)
    recvd = 0
    with open(ofile,'wb') as flashfile:
        while(recvd < size):
            dat = sock.recv(2)
            recvd += 2
            flashfile.write(dat)
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Flashtality Python Script")
    parser.add_argument("-o","--ofile",type=str,help="File to output resulting read to")
    parser.add_argument("-p","--port",type=int,help="Port to connect to")
    parser.add_argument("-i","--ip",type=str,help="IP address of ESP32")
    parser.add_argument("-a","--startaddress",type=str,help="Address to start dumping from")
    parser.add_argument("-s","--size",type=str,help="Size of dump ... ")
    args = parser.parse_args()
    esp_socket = esp_connect(args.ip,args.port)
    dump_flash(esp_socket,int(args.startaddress,16),int(args.size,16),args.ofile)
