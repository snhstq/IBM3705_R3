/* Copyright (c) 202?, Henk Stegeman and Edwin Freekenhorst

   Permission is hereby granted, free of charge, to any person obtaining a
   copy of this software and associated documentation files (the "Software"),
   to deal in the Software without restriction, including without limitation
   the rights to use, copy, modify, merge, publish, distribute, sublicense,
   and/or sell copies of the Software, and to permit persons to whom the
   Software is furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
   HENK STEGEMAN AND EDWIN FREEKENHORST BE LIABLE FOR ANY CLAIM, DAMAGES OR
   OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
   ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
   DEALINGS IN THE SOFTWARE.
   ---------------------------------------------------------------------------

   i3705_sdlc.c: IBM 3720 SDLC Primary Station simulator

   SLDC frame
    <-------------------------------- BLU ----------------------------->
   layout:         |   FCntl   |
   +-------+-------+-----------+-------//-------+-------+-------+-------+
   | BFlag | FAddr |Nr|PF|Ns|Ft| ... Iframe ... | Hfcs  | Lfcs  | EFlag |
   +-------+-------+-----------+-------//-------+-------+-------+-------+
*/

#include <stdbool.h>
#include "sim_defs.h"
#include "i3705_defs.h"
#include <ifaddrs.h>
#include <pthread.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>                     /* Added for debugging       */

#define MAX_LINES       4              /* Maximum of lines          */
#define BUFLEN_LINE     16384          /* Line Send/Receive buffer  */
#define LINEBASE        20             /* SDLC lines start at 20    */
                                       /* Make sure this matches the Buffer of the attached device */
#define TX              0              /* lines state definitions   */
#define RX              1
#define DISC            2              /* 3274 is disconnected      */
#define CONN            3              /* 3274 is connected         */

struct SDLCLine {
   int      line_fd;
   int      line_num;
   int      line_stat;
   int      d3274_fd;
   int      epoll_fd;
   uint8_t  SDLC_rbuf[BUFLEN_LINE];    // Received data buffer
   uint8_t  SDLC_tbuf[BUFLEN_LINE];    // Transmit data buffer
} *sdlcline[MAX_LINES];

extern FILE *S_trace;                  // Externals for debugging
extern uint16_t Sdbg_reg;
extern uint16_t Sdbg_flag;

// Host ---> PU request buffer
extern uint8 BLU_req_buf[MAX_LINES][BUFLEN_LINE];  // DLC header + TH + RH + RU + DLC trailer
extern int   BLU_req_ptr[MAX_LINES];   // Offset pointer to BLU
extern int   BLU_req_len[MAX_LINES];   // Length of BLU request
extern int   BLU_req_stat[MAX_LINES];  // BLU tx buffer state: FILLED or EMPTY
// PU ---> Host response buffer
extern uint8 BLU_rsp_buf[MAX_LINES][BUFLEN_LINE];  // DLC header + TH + RH + RU + DLC trailer
extern int   BLU_rsp_ptr[MAX_LINES];   // Offset pointer to BLU
extern int   BLU_rsp_len[MAX_LINES];   // Length of BLU response
extern int   BLU_rsp_stat[MAX_LINES];  // BLU rx buffer state: FILLED or EMPTY

int j;                                 // Line pointer
int rcv_cnt;                           // Number of bytes received
int SendSDLC(int j);
int ReadSDLC(int j);


//************************************************************************
//   Thread to handle SDLC frames between scanner and the 3274 emulator  *
//************************************************************************
void *SDLC_thread(void *arg) {
   int    devnum;                  /* device nr copy for convenience    */
   int    sockopt;                 /* Used for setsocketoption          */
   int    event_count;             /* # events received                 */
   int    rc, rc1;                 /* return code from various rtns     */
   struct sockaddr_in  sin, *sin2; /* bind socket address structure     */
   struct ifaddrs *nwaddr, *ifa;   /* interface address structure       */
   char   *ipaddr;
   struct epoll_event event, events[MAX_LINES];

   fprintf(stderr, "\n\rSDLC: Thread %ld started succesfully...", syscall(SYS_gettid));
   // Assign thread to a core
   // core_id = 1 (CPU), 2 (SCAN), 3 (SDLC)
   int core_id = 3;
   int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
   if ((core_id > 0) && (core_id <= num_cores)) {
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      CPU_SET(core_id, &cpuset);
      pthread_t current_thread = pthread_self();
      pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
      fprintf(stderr, "\n\rSDLC: Thread assigned to core #%1d.", core_id);
   }

   for (int j = 0; j < MAX_LINES; j++) {
      sdlcline[j] = malloc(sizeof(struct SDLCLine));
      sdlcline[j]->line_num  = j;
      sdlcline[j]->line_stat = CONN;
      BLU_rsp_len[j] = 0;
   }  // End for j = 0

   getifaddrs(&nwaddr);      /* Get network address */
   for (ifa = nwaddr; ifa != NULL; ifa = ifa->ifa_next) {
      if (ifa->ifa_addr->sa_family == AF_INET && strcmp(ifa->ifa_name, "lo")) {
         sin2 = (struct sockaddr_in *) ifa->ifa_addr;
         ipaddr = inet_ntoa((struct in_addr) sin2->sin_addr);
         if (strcmp(ifa->ifa_name, "eth")) break;
      }
   }
   printf("\n\rSDLC: Using network Address %s on %s for PU connections.", ipaddr, ifa->ifa_name);

   // *******************************************************************
   //   Open a TCPIP socket(s) for each 3274 line
   // *******************************************************************
   for (j = 0; j < MAX_LINES; j++) {
      if ((sdlcline[j]->line_fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0)) == -1)
         printf("\n\rSDLC-%d: Endpoint creation for 3274 failed with error %s ", j, strerror(errno));

      /* Reuse the address regardless of any */
      /* spurious connection on that port.   */
      sockopt = 1;
      setsockopt(sdlcline[j]->line_fd, SOL_SOCKET, SO_REUSEADDR, (void*)&sockopt, sizeof(sockopt));

      // Bind the socket
      sin.sin_family = AF_INET;
      sin.sin_addr.s_addr = inet_addr(ipaddr);
      sin.sin_port = htons(37500 + LINEBASE + j);        // <=== port related to line number

      if (bind(sdlcline[j]->line_fd, (struct sockaddr *)&sin, sizeof(sin)) < 0) {
         printf("\n\rSDLC-%d: Bind line-%d socket failed",j, j);
         free(sdlcline[j]);
         exit(EXIT_FAILURE);
      }
      // Listen and verify...
      if ((listen(sdlcline[j]->line_fd, 10)) != 0) {
         printf("\n\rSDLC-%d: Line-%d Socket listen failed %s", j, j, strerror(errno));
         free(sdlcline[j]);
         exit(-1);
      }
      // Add polling events for the port
      sdlcline[j]->epoll_fd = epoll_create(1);
      if (sdlcline[j]->epoll_fd == -1) {
         printf("\n\rSDLC-%d: Failed to created the line-%d epoll file descriptor", j, j);
         free(sdlcline[j]);
         exit(-2);
      }
      event.events = EPOLLIN;
      event.data.fd = sdlcline[j]->line_fd;
      if (epoll_ctl(sdlcline[j]->epoll_fd, EPOLL_CTL_ADD, sdlcline[j]->line_fd, &event) == -1) {
         printf("\n\rSDLC-%d: Add polling event failed for line-%d with error %s ",
                  j, j, strerror(errno));
         close(sdlcline[j]->epoll_fd);
         free(sdlcline[j]);
         exit(-3);
      }
      printf("\n\rSDLC-%d: line ready, waiting for connection on TCP port %d", j, 37500 + LINEBASE + j );
   }


   // ******************************************************************************
   //   Check forever all lines for in- or outgoing data or connection request.
   // ******************************************************************************
   while (1) {
      for (j = 0; j < MAX_LINES; j++) {

         // ************************************************************************
         //   Transmitting (CCU ---> line) SDLC frame(s) in BLU buffer
         // ************************************************************************
         // Check if BLU_req_buf (Tx) is filled by scanner.
         if ((sdlcline[j]->line_stat == CONN) && (BLU_req_stat[j] == FILLED)) {  // Any data from host ?
            rc = SendSDLC( j );          // Transfer buffer content to 3274
            if (rc <  0) {               // No connection ?
               sdlcline[j]->line_stat = DISC;
               continue;
            }
            if (rc == 0) {               // All data send to 3274 ?
               BLU_req_len[j]  = 0;      // Reset Tx buffer length
               BLU_req_stat[j] = EMPTY;  // Request is processed; set buffer is empty.
               sdlcline[j]->line_stat = CONN; // Line turnaround
               continue;
            }
            if (rc  > 0) {               // ???
               continue;
            }
         }  // End if FILLED

         // ************************************************************************
         //   Receiving (CCU <--- line) SDLC frame(s) in BLU buffer
         // ************************************************************************
         // Check for any response from 3274
         if ((sdlcline[j]->line_stat == CONN) && (BLU_rsp_stat[j] == EMPTY)) {  // Any data from 3274
            rc = ReadSDLC( j );          // Transfer 3274 response to BLU_rsp_buf
            if (rc <  0) {               // No connection ?
               sdlcline[j]->line_stat = DISC;
               continue;
            }
            if (rc == 0) {               // No data received yet.
               BLU_rsp_stat[j] = EMPTY;
               continue;
            }
            if (rc  > 0) {               // Data received from 3274 ?
               BLU_rsp_len[j] = rc;
               BLU_rsp_stat[j] = FILLED; // Indicate there is data in the response buffer
               sdlcline[j]->line_stat = CONN;
               continue;
            }
         }  // End sdlcline[j]->line_stat

         // ********************************************************************
         //   Poll briefly for a TCP_IP connect request.
         //   If a connect request is received, proceed with connect/accept the request.
         // ********************************************************************
         if (sdlcline[j]->line_stat == DISC) {
            event_count = epoll_wait(sdlcline[j]->epoll_fd, events, 1, 25);
            for (int i = 0; i < event_count; i++) {
               sdlcline[j]->d3274_fd = accept (sdlcline[j]->line_fd, NULL, 0);
               if (sdlcline[j]->d3274_fd < 1) {       // No connection ?
                  printf("\n\rSDLC-%d: Accept failed for line %s", j, strerror(errno));
               } else {
                  printf("\n\rSDLC-%d: PU connected to line", j);
                  sdlcline[j]->line_stat = CONN;      // Default state after connect is TX
               }  // End if sdlcline[j]->d3274_fd
            }  // End for int i
         }  // End if (sdlcline[j]->line_stat

      }  // End for line = 0 --> MAXLINES (next line please)
      usleep(100);
   }  // End while(1)

   return NULL;
}  // End of *SDLC_thread


//*********************************************************************
// Send SDLC frame(s) in BLU_req_buf to the 3274                      *
// If an error occurs, the TCPIP connection will be closed            *
//*********************************************************************
int SendSDLC(int j) {
   register char *s;
   int Pflag = FALSE;                    // SDLC Poll bit flag
   int Fptr, frame_len;                  // SDLC frame pointer & length
   int i, rc;

   if ((Sdbg_flag == ON) && (Sdbg_reg & 0x04)) {  // Trace BLU activities ?
      fprintf(S_trace, "\n\n\r#04L%1d> SDLC: Received %d bytes request from scanner."
                       "\n\r#04L%1d> SDLC: Request Buffer: "
                       "\n\r#04L%1d> SDLC: ", j, BLU_req_len[j], j, j);
      for (i = 0; i < BLU_req_len[j]; i++) {
         fprintf(S_trace, "%02X ", (int) BLU_req_buf[j][i] & 0xFF);
         if ((i + 1) % 32 == 0)
            fprintf(S_trace, " \n\r#04L%1d> SDLC: ", j);
      }
   }  // End if Sdbg_flag

   // Search for SDLC frames in BLU_req_buf, process it and when a Poll bit is found: Return.
   Pflag = FALSE;
   Fptr = 0;
   if ((BLU_req_buf[j][Fptr] == 0x00) ||         // If modem clocking is used
       (BLU_req_buf[j][Fptr] == 0xAA))           // skip 1st char (0xAA or 0x00)
      Fptr = 1;

   if ((Sdbg_flag == ON) && (Sdbg_reg & 0x04))   // Trace BLU activities ?
      fprintf(S_trace, "\n\r#04L%1d> SDLC: Sending %d bytes to 3274.",
                        j, BLU_req_len[j]-Fptr);

   if (sdlcline[j]->d3274_fd < 1)        // PU connected ? No: return -1
      return (-1);
   // ******************************************************************
   rc = send(sdlcline[j]->d3274_fd, &BLU_req_buf[j][Fptr], BLU_req_len[j]-Fptr, 0);
   // ******************************************************************
   if (rc < 0) {
      printf("\n\rSDLC-%d: [SendSDLC] Send failed with error %s",
               j, strerror(errno));
      close (sdlcline[j]->d3274_fd);
      sdlcline[j]->d3274_fd = 0;
      printf("\n\rSDLC-%d: [SendSDLC] PU disconnected from line", j);
      return(-1);                        // PU connection lost
   } else {
      return(0);
   } // End if (rc < 0)
   usleep(100);

}


//*********************************************************************
// Read SDLC frame(s) from the 3274                                   *
// If an error occurs, the connection will be closed                  *
//*********************************************************************
int ReadSDLC(int j) {
   int rc, rcv_cnt;
   int Fptr = 0;
   rcv_cnt = 0;

   if (sdlcline[j]->d3274_fd > 0)        // Check if any data received
      rc = ioctl(sdlcline[j]->d3274_fd, FIONREAD, &rcv_cnt);
   if (rc < 0) {
      close (sdlcline[j]->d3274_fd);     // Close the line.
      sdlcline[j]->d3274_fd = 0;
      printf("\n\rSDLC-%d: [ReadSDLC] PU disconnected from line.", j);
      return(-1);                        // Connection lost
   }  // End if (sdlcline[j]...

   if (rcv_cnt > 0) {
      // ******************************************************************
      BLU_rsp_len[j] = read(sdlcline[j]->d3274_fd, &BLU_rsp_buf[j][Fptr], BUFLEN_LINE);
      // ******************************************************************

      if ((Sdbg_flag == ON) && (Sdbg_reg & 0x04) && (BLU_rsp_len[j] > 0)) {
         fprintf(S_trace, "\n\r#04L%1d< SDLC: Received %d bytes response from 3274. "
                          "\n\r#04L%1d< SDLC: Response Buffer: "
                          "\n\r#04L%1d< SDLC: ", j, BLU_rsp_len[j], j, j);
         for (int i = 0; i < BLU_rsp_len[j]; i++) {
            fprintf(S_trace, "%02X ", (int) BLU_rsp_buf[j][i] & 0xFF);
            if ((i + 1) % 32 == 0)
               fprintf(S_trace, "\n\r#04L%1d< SDLC: ", j);
         }
         fprintf(S_trace, "\n\r#04L%1d< SDLC: Sending %d bytes to scanner.",
                           j, BLU_rsp_len[j]);
      }  // End if Sdbg_flag

      return(BLU_rsp_len[j]);            // Received data present in BLU_rsp_buf
   } else {
      return(0);                         // No data received (yet)
   }  // End if (rcv_cnt > 0)

}  // End of ReadSDLC

