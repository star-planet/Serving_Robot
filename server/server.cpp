#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <pthread.h>

#define PORT 8080

const int MAX_LINE = 2048;
const int BACKLOG = 10;
const int LISTENQ = 6666;
const int MAX_CONNECT = 20;


// 클라이언트로부터 메시지 수신
void *recv_message(void *fd)
{
	int sockfd = *(int *)fd;
	while(1)
	{
		char buf[MAX_LINE];
		memset(buf , 0 , MAX_LINE);
		int n;
		if((n = recv(sockfd , buf , MAX_LINE, 0)) == -1)
		{
			perror("recv error.\n");
			exit(1);
		}
		buf[n] = '\0';

        // 클라이언트가 갑자기 접속을 끊은 경우 처리 
        if (n==0)
        {
			printf("Client closed.\n");
			close(sockfd);
			exit(1);
        }
		printf("Client: %s", buf);
	}
}

int main(int argc, char* argv[])
{
	int listenfd , connfd;
	socklen_t clilen;
	pthread_t thread[2];

	struct sockaddr_in servaddr , cliaddr;
	
	if((listenfd = socket(AF_INET , SOCK_STREAM , 0)) == -1)
	{
		perror("socket error.\n");
		exit(1);
	}


	bzero(&servaddr , sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	servaddr.sin_port = htons(PORT);

    // bind 오류 해결. 사용했던 ip라고 뜨는 에러. 
    int val = 1;    
    if (setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, (char *) &val, sizeof val) < 0) {
        perror("setsockopt");
        close(listenfd);
	    return -1;
    }

	if(bind(listenfd , (struct sockaddr *)&servaddr , sizeof(servaddr)) < 0)
	{
		perror("bind error.\n");
		exit(1);
	}

	if(listen(listenfd , LISTENQ) < 0)
	{
		perror("listen error.\n");
		exit(1);
	}

	clilen = sizeof(cliaddr);
	
		if((connfd = accept(listenfd , (struct sockaddr *)&cliaddr , &clilen)) < 0)
		{
			perror("accept error.\n");
			exit(1);
		}

		printf("server: got connection from %s\n", inet_ntoa(cliaddr.sin_addr));
		
	while(1){
		// 메시지 수신하는 스레드 생성
		if(pthread_create(&thread[0] , NULL , recv_message, &connfd) == -1)
		{
			perror("pthread create error.\n");
			exit(1);
		}

		char msg[MAX_LINE];
		memset(msg , 0 , MAX_LINE);
		if(fgets(msg , MAX_LINE , stdin) != NULL)	
		{	
			send(connfd , msg , strlen(msg) , 0);
		}
	}

	return 0;
}
