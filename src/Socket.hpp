#include <winsock2.h>  
#pragma comment(lib, "ws2_32.lib")  

int MySend(string buf)
{
	WSADATA         wsd;            
	SOCKET          sHost;          
	SOCKADDR_IN     servAddr;       
	int             retVal;       

	if (WSAStartup(MAKEWORD(2, 2), &wsd) != 0) 	{
		cout << "WSAStartup failed!" << endl;
		return -1;
	}

	sHost = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (INVALID_SOCKET == sHost)
	{
		cout << "socket failed!" << endl;
		WSACleanup();//释放套接字资源  
		return  -1;
	}

	servAddr.sin_family = AF_INET;
	servAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
	servAddr.sin_port = htons(11229);
 
	retVal = connect(sHost, (LPSOCKADDR)&servAddr, sizeof(servAddr));
	if (SOCKET_ERROR == retVal)	{
		cout << "connect failed!" << endl;
		closesocket(sHost); 
		WSACleanup();       
		return -1;
	}

	retVal = send(sHost, buf.c_str(), buf.size(), 0);
	if (SOCKET_ERROR == retVal)	{
		cout << "send failed!" << endl;
		closesocket(sHost); 
		WSACleanup();       
		return -1;
	}
	retVal = send(sHost, "", 0, 0);

	//退出  
	closesocket(sHost); 
	WSACleanup();       
	return 0;
}