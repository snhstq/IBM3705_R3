/* Copyright (c) 2021, Henk Stegeman & Edwin Freekenhorst
   (c) Copyright Max H. Parke, 2007-2012

   Permission is hereby granted, free of charge, to any person obtaining a
   copy of this software and associated documentation files (the "Software"),
   to deal in the Software without restriction, including without limitation
   the rights to use, copy, modify, merge, publish, distribute, sublicense,
   and/or sell copies of the Software, and to permit persons to whom the
   Software is furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
   ROBERT M SUPNIK BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
   IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
   ----------------------------------------------------------------------------

   i327x_sdlc.h: IBM 3705 SDLC/SNA definitions
*/

/* General */
#define RX             0       // Rx = client -> 3705
#define TX             1       // Tx = 3705 -> client
#define RESET          9       // Line in idle state
#define INITM          0
#define NRM            1       // Normal Response Mode
#define NDM            2       // Normal Disconnect Mode

/* SDLC frame layout (BLU)
   layout:         |   FCntl   |
   |---0---+---1---+-----2-----+---3---//----n--+-------+-------+-------|
   | BFlag | FAddr |Nr|PF|Ns|Ft|.. Iframe/PIU ..| Hfcs  | Lfcs  | EFlag |
   |-------+-------+-----------+-------//-------+-------+-------+-------|
*/
#define BUFLEN_3274     16384          /* 3274 Send/Receive buffer  */
/**************************/
/* SDLC frame header defs */
/**************************/
#define BFlag          0
#define FAddr          1
#define FCntl          2
#define Nr           ((BLU_req_buf[FCntl] >> 5) & 0x7)
#define Ns           ((BLU_req_buf[FCntl] >> 1) & 0x7)
#define PF           ((BLU_req_buf[FCntl] >> 4) & 0x1)
#define Ft            (BLU_req_buf[FCntl] & 0x01)
#define CPoll          0x10
#define CFinal         0x10
#define IFrame         3               // Offset IFrame
#define PIU            3               // Offset PIU in BLU
#define Hfcs           3               // Offset from BFlag
#define Lfcs           4               //  if no PIU
#define EFlag          5               //  (Plen = 0)

/* Used for Unnumbered cmds/resp */
#define UNNUM          0x03
#define SNRM           0x83            // CommandS
#define DISC           0x43
#define XID2           0xAF
#define UA             0x63            // Responses
#define DM             0x0F
#define FRMR           0x87
#define TEST           0xE3
#define XID            0xCF

/* Used for Supervisory cmds/resp */
#define SUPRV          0x01
#define RR             0x01
#define RNR            0x05
#define REJ            0x09

/* Used for Information frame cmds/resp */
#define IFRAME         0x00

/* SNA FID2 frame layout (PIU)
  /-|-3--+-4--+-5--+-6--+-7--+-8--|-9--+-10-+-11-|-12-+--//-nn-|-/
  ~ |FID2|resv|DAF |OAF | seq nr. |RH0 |RH1 |RH2 |RU0...    ...| ~
  /-|----+----+----+----+----+----|----+----+----|----+--//----|-/
      ^
      | PIU starts here
*/
/**************************/
/* SNA FID2  header defs  */
/**************************/
// Transmission Header (TH)
#define FD2_TH_len   6                 // Length FID2 TH
#define FD2_TH_0     PIU+0             // FID type 2
#define FD2_TH_1     PIU+1             // Reserved
#define FD2_TH_daf   PIU+2             // Destination addr
#define FD2_TH_oaf   PIU+3             // Origin addr
#define FD2_TH_scf0  PIU+4             // Sequence Count (H)
#define FD2_TH_scf1  PIU+5             // Sequence Count (L)
// Req/Resp Header (RH)
#define FD2_RH_len   3                 // Length FID2 RH
#define FD2_RH_0     PIU+6             // x....... 0 = Req, 1 = Resp.
#define FD2_RH_1     PIU+7             // x.x..... 1 = Def Resp.
#define FD2_RH_2     PIU+8             // Reserved
// Req/Resp Unit (RU)
#define FD2_RU_0     PIU+9             // DFC, NC, SC Req codes
#define FD2_RU_1     PIU+10
#define FD2_RU_2     PIU+11

