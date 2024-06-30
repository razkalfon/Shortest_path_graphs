//Nir Ohayon - 206542953
//Raz kalfon - 213006083


#include "std_lib_facilities.h"
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <string>
#include <sys/wait.h>

int main(int argc, char **argv){
    if (argc != 5) {
        error("Please enter 4 elements <IP> <port> <Source> <Destination>");
    }
    string ip = argv[1];
    string port = argv[2];
    string v1 = argv[3];
    string v2 = argv[4];

    int fd=socket(AF_INET,SOCK_STREAM,0);
    sockaddr_in addr={0};
    addr.sin_family=AF_INET;
    addr.sin_addr.s_addr=inet_addr("127.0.0.1");
    addr.sin_port=htons(stoi(port));
    connect(fd,(sockaddr*)&addr,sizeof(addr));
    string v1v2=v1;
    v1v2+=" ";
    v1v2+=v2;

    write(fd,&v1v2[0],v1v2.size());

    // Read the response from the server
    char buffer[256] = {0};
    read(fd, buffer, 256);

    // Print the response
    cout << "Shortest path: " << buffer << endl;

    close(fd);
    return 0;
}
