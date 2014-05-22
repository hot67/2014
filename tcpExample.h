/*
 * For CheesyVisionServer (TCP Server).
 * Configuration of Server
 */
#ifndef _TCPEXAMPLE_H
#define _TCPEXAMPLE_H

/* tcpExample.h - header used by both TCP server and client examples */  
 
/* defines */ 
#define SERVER_PORT_NUM         5001   /* server's port number for bind() */ 
#define SERVER_WORK_PRIORITY    100    /* priority of server's work task */ 
#define SERVER_STACK_SIZE       10000  /* stack size of server's work task */ 
#define SERVER_MAX_CONNECTIONS  4      /* max clients connected at a time */ 
#define REQUEST_MSG_SIZE        1024   /* max size of request message */ 
#define REPLY_MSG_SIZE          500    /* max size of reply message */ 
 
/* structure for requests from clients to server */ 
struct request 
    { 
    int reply;                         /* TRUE = request reply from server */  
    int msgLen;                        /* length of message text */ 
    char message[REQUEST_MSG_SIZE];    /* message buffer */  
    };

#endif
