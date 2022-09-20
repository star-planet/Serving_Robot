#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <signal.h>
#include <sys/wait.h>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <thread>
#include <iostream>
#include <fstream>

#define BUFSIZE 4096
#define PORT 8081

using namespace std;

void error_handling(char* message);

class Socket{
    
    protected:
    int serv_sock;

    public:
    Socket(){
        serv_sock = socket(AF_INET, SOCK_STREAM, 0);
    }
};

class ServerSocket : public Socket{    
    private:
    int clnt_sock;
    pthread_t tid;
    char temp[20];

    public:
    bool BindSocket(){
        struct sockaddr_in addr;
        bzero((char*)&addr, sizeof(addr));
        
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(PORT);

        int val = 1;    
        if (setsockopt(serv_sock, SOL_SOCKET, SO_REUSEADDR, (char *) &val, sizeof val) < 0) {
            perror("setsockopt");
            close(serv_sock);
            return -1;
        }

        int flag = bind(serv_sock, (struct sockaddr*) &addr, sizeof(addr));
        if (flag == -1)
            return false;
        else
            return true;
    }

    bool ListenSocket(int num){
        int flag = listen(serv_sock, num );
        if (flag == -1)
            return false;
        else
            return true;
    }

    bool AcceptSocket(){
        struct sockaddr_in addr;
        socklen_t addr_size;
        addr_size = sizeof(addr);
        clnt_sock = accept(serv_sock, (struct sockaddr *)&addr, &addr_size);
        
        if (clnt_sock == -1)
            return false;
        else
            inet_ntop(AF_INET, &addr.sin_addr.s_addr, temp, sizeof(temp));
            cout << "[ INFO] Client IP: " << temp << endl; 
            return true;
    }

    bool SendMsg(){
        while(1){
            char buf[BUFSIZE];
            memset(buf , 0 , BUFSIZE);

            ifstream input("/home/hyun/socket3");
            input >> buf;
            input.close();

            int n;
            if((n = write(clnt_sock , buf , strlen(buf))) == -1){
                perror("send error.\n");
            }

            buf[n] = '\0';

            if(n == 1){
                perror("[ INFO] Status");
                cout << "[ INFO] Server: " << buf << "\n" << endl;
                ofstream output("/home/hyun/socket3");
                output.close();                
            }
        }
    }

    static void* ReceiveMsg(void* data){
        int sockfd = *(int*)data;
        while (1){
            char buf[BUFSIZE];
            memset(buf , 0 , BUFSIZE);
            int n;
            if((n = recv(sockfd , buf , BUFSIZE, 0)) == -1){
                perror("recv error.\n");
                exit(1);
            }

            buf[n] = '\0';

            if (n == 0){
                printf("Client closed.\n");
                close(sockfd);
                exit(1);
            }

            perror("[ INFO] Status");
            printf("[ INFO] Client: %s\n", buf);
        }
    }

    void ReceiveMsg_Thread(){
        pthread_create(&tid, NULL, ReceiveMsg, &clnt_sock);
    }
    
    void CloseListenSocket(){
        close(serv_sock);
    }
    
    void CloseAcceptSocket(){
        close(clnt_sock);
    }


};

int main ( int argc, char * argv[]){
    cout << " ______________________________ " << endl;
    cout << "|                              |" << endl;
    cout << "|      MCA TCP/IP Server       |" << endl;
    cout << "|______________________________|\n" << endl;
    

    int state;
    pid_t pid;

    ServerSocket serv_sock;

    if(!(serv_sock.BindSocket()))
        error_handling("bind() error.\n");

    if(!(serv_sock.ListenSocket(5)))
        error_handling("Listen() error.\n");

    while(1){
        if(!(serv_sock.AcceptSocket()))
            continue;
        else
            puts("[ INFO] Server: New client connected.\n");

        pid = fork();

        if (pid == 0){
            serv_sock.ReceiveMsg_Thread();
            serv_sock.SendMsg();
            return 0;
        }
    }
    serv_sock.CloseListenSocket();
    return 0;   
}

void error_handling(char* message){
    fputs(message, stderr);
    fputc('\n', stderr);
    exit(1);
}