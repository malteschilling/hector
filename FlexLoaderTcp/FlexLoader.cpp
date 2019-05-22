/* FlexLoader PC application (c) 2010-2013 Mattias Schaeffersmann
 */
#include <stdlib.h>
#include <string>
#include <err.h>
#include <errno.h>
#include <stdint.h>
#include <fcntl.h>

#if !defined(__APPLE__)
#include <malloc.h>
#else
#include <malloc/malloc.h>
#endif

#include <sys/wait.h>
#include <stdio.h>
#include <sys/select.h>
#include <stddef.h>
#include <unistd.h>
#include <time.h>
#include <sys/stat.h>
#include <termios.h>
#include <sys/ioctl.h>

#include <vector>

#include "FlexLoader.hpp"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/optional.hpp>
#include <boost/system/error_code.hpp>


// This function seems to only output information regarding a received function
// b points to the buffer of the payload
// l defines the length of the buffer
static void ppack(uint8_t *b,int l){
	if(l<1){
		fprintf(stderr,"Nothing\n");
		return;
	}
	switch(b[0]){
		case 0:
			fprintf(stderr,"dummy\n");
			break;
		default:
			fprintf(stderr,"unknown packet:");
			int i;
			for(i=0;i<l;i++){
				fprintf(stderr,"%02x",b[i]);
			};
			fprintf(stderr,"\n");
	}
}

void setResult(int* bytesTransferedDestination, const boost::system::error_code& error, size_t bytesTransferred){
	if(!error){
		*bytesTransferedDestination=bytesTransferred;
	}
}

void timeoutNotifier(bool* timeoutOrCancelled, const boost::system::error_code& error){
	*timeoutOrCancelled=true;
} 



// Read a message from the serial port
// fd is a handler to the serial port
// buf is the buffer the data should be written to
// l is the number of bytes that were read
// pstart is the position of the payload start (the protocol id
// plen is the number of bytes of the payload
static int rpack(boost::asio::ip::tcp::socket* socket,uint8_t*buf,int*l,int*pstart,int*plen){
	int sel;
	int ioam=0;
	int woff=0; // write offset in the data buffer used for storing data
	int avl;
	int req=8; // The number of bytes that should be received
	int ratt=0; // number of unsuccessful read attempts
	int packstart=0;
	int packlen; // The length of the packet
	int packok=0;
	//struct timeval timev;
	//fd_set fdset; // 
	//FD_ZERO(&fdset); // "clears a set"
	while(!packok){ // loop until a message was received
		if(ratt>5){
			return 0;
		}
		//FD_SET(fd,&fdset); // this probably connects the serial port with the fd_set such that it can be used in order to find out whether the port is writable
		//timev.tv_sec=0;
		//timev.tv_usec=100000;
		//sel=select(fd+1,&fdset,NULL,NULL,&timev); // select() and pselect() allow a program to monitor multiple file descriptors, waiting until one or more of the file descriptors become "ready" for some class of I/O operation (e.g., input possible). A file descriptor is considered ready if it is possible to perform the corresponding I/O operation (e.g., read(2)) without blocking.
		// select() uses a timeout that is a struct timeval (with seconds and microseconds)
		// select() returns the number of file descriptors contained in the three returned descriptor sets
		// select() returns -1 in case of an error
		sel=1; // since the tcp port has already been opened, let's assume that everything is all right. 
		if(sel<0){ // if an error has occured
			fprintf(stderr,"Critical error\n");return 0;
		}
		if(!sel){ // if no file descriptor was found, try again and increase the number of usuccessful attempts
			fprintf(stderr,"?");
			ratt++;
			continue;
		}
		
		bool timeoutOrCancelled=false; 
		boost::asio::deadline_timer timer(socket->get_io_service()); 
		timer.expires_from_now(boost::posix_time::milliseconds(100)); 
		timer.async_wait(boost::bind(timeoutNotifier, &timeoutOrCancelled, boost::asio::placeholders::error)); 
		
		
		boost::asio::async_read(*socket, boost::asio::buffer(&(buf[woff]),req),
			boost::bind(&setResult, 
				&ioam,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred)
		);
		
		while (ioam==0 && timeoutOrCancelled==false) { 
			usleep(100);
		}
		if (ioam!=0){
			timer.cancel(); 
			while(timeoutOrCancelled==false){
				usleep(1);
			};
		}else if (timeoutOrCancelled){
			socket->cancel(); 
			break;
		};
		
		
		if(ioam<0){ // if an error occured
			fprintf(stderr,"Critical error\n");return 0;
		}
		if(!ioam){ // if no data was received, increase the number of unsuccessful read attempts
			fprintf(stderr,"Error\n");
			ratt++;
			continue;
		}
		woff+=ioam; // shift the write offset by the number of received bytes
		while((avl=woff-packstart)>=8){ // if there are at least eight bytes in the read buffer that have not been discarded
			if(buf[packstart+2]&0x10){ // if seems to be a long message
				if(buf[packstart+4]!=0xDB){ // if the dummy header payload for a long message is wrong, move the position of the read offset one byte further (this will discard the first byte) and set the number of required bytes to 1 in order to gain a full short message or at least the first eight bytes of a long message
					packstart++;
					req=1;
					continue;
				}
				packlen=buf[packstart+3]; // get the length of the packet
				if(avl<packlen){ // if the number of undiscarded bytes in the read buffer is less than the assumed length of the message
					req=packlen-avl; // set the difference as the number of bytes that should be read
					break;
				}
				if((buf[packstart+packlen-2]!=0x66)||(buf[packstart+packlen-1])){ // if the dummy payload crcs don't fit, discard the first byte and continue
					packstart++;
					req=1;
					continue;
				}
				*pstart=5; // set the start of the payload (the protocol id byte)
				*plen=packlen-7; // the number of payload bytes
				*l=packlen; // the number of bytes in the whole message
			}else{ // if it is a short message
				if(buf[packstart+7]!=0xAA){ // if the dummy crc does not fit, continue with the next byte
					packstart++;
					req=1;
					continue;
				}
				*pstart=3; // set the start of the payload (the protocol id byte)
				*plen=4; // the number of payload bytes
				*l^=*l; // every time lots of fun: setting l to zero; it is assumed that this means that a short message was received
			}
			packok=1; // A message has been successfully received
			break;
		}
		if(woff>0){// if some data has been received.. I have no idea what this might do.
			int remaining=woff-packstart; // compute the remaining number of bytes
			uint64_t*src=(uint64_t*)&(buf[packstart]);
			uint64_t*dst=(uint64_t*)buf;
			while(packstart<woff){
				*dst++=*src++;
				packstart+=8;
			}
			woff=remaining;
			packstart^=packstart; // setting packstart to zero
		}
	}
	return 1;
}
// It seems that this function is responsible for writing a message. Probably without trying to receive an answer.
// fd seems to be the handle to the serial port.
// wbuf points to the location of the data that should be send.
// wlen defines the length of the message (in bytes)
static int wonce(boost::asio::ip::tcp::socket* socket,uint8_t*wbuf,int wlen){
	int iores;
	if(!wlen){ // if the length of the packet is not specified, find out how long it is
		if(wbuf[2]&0x10){
			wlen=wbuf[3];
		}else{
			wlen=8;
		}
	}
	if(wbuf[2]&0x10){
		wbuf[3]=wlen;
		wbuf[4]=0xDB;
		wbuf[wlen-2]=0x66;
		wbuf[wlen-1]=0;
	}else{
		wbuf[7]=0xAA;
	}
	iores=boost::asio::write(*socket,boost::asio::buffer(wbuf,wlen));
	//fflush(NULL);
	if(iores < 0){ // if the message could not be sent
		fprintf(stderr,"Critical error\n");
		return 0;
	}
	return 1;
}

// Send a message via the serial interface and try to receive a reply
// fd seems to be the handle to the serial port.
// wbuf points to the location of the data that should be send.
// wlen defines the length of the message (in bytes)
// rbuf is the write buffer that should be used for a reply
// rlen is the address of the memory into which the length of the received reply in bytes
// rps points to the start of the payload (the protocol id)
// rpl is the address of the memory into which the length of the reply will be written
static int wrr(boost::asio::ip::tcp::socket* socket,uint8_t*wbuf,int wlen,uint8_t*rbuf,int*rlen,int*rps,int*rpl){
	int iores;
	if(!wlen){// if wlen is zero
		if(wbuf[2]&0x10){
			wlen=wbuf[3];
		}else{
			wlen=8;
		};
		// replaces: wlen=(wbuf[2]&0x10)?wbuf[3]:8;
	}
	if(wbuf[2]&0x10){ // if the message's long flag is set
		wbuf[3]=wlen; // fill the length information
		wbuf[4]=0xDB; // set the dummy header crc
		wbuf[wlen-2]=0x66; // set the dummy payload crc
		wbuf[wlen-1]=0; // set the dummy payload crc
	}else{
		wbuf[7]=0xAA; // set the dummy crc
	};
	int retries=5;
	do{
		iores=boost::asio::write(*socket,boost::asio::buffer(wbuf,wlen)); // write the number of bytes specified by wlen from the memory specified by wbuf to the serial port
		//fflush(NULL); // For output streams, fflush() forces a write of all user-space buffered data for the given output or update stream via the stream's underlying write function. For input streams, fflush() discards any buffered data that has been fetched from the underlying file, but has not been consumed by the application. The open status of the stream is unaffected. 
		// If the stream argument is NULL, fflush() flushes all open output streams.
		if(iores<0){ // If the write was unsuccessful
			fprintf(stderr,"Critical error\n");
			return 0;
		}
		if(rpack(socket,rbuf,rlen,rps,rpl)){ // try to receive a reply; if that fails, try again. If it works, return
			return 1;
		};
	}while(retries-->0); // if the decremented value of retries is bigger than zero, try again...
	return 0;
}

// Probably, this function is used to create packages that contain the firmware. 
// buf points to the memory where the message should be written to
// id is the id of the client this messaage will be sent to
static void pdatpak(uint8_t*buf,uint8_t id,uint8_t*fdata,uint32_t off){
	*((uint64_t*)buf)=0x2209008E100200ll|id; // destination=id; source=2; flags=long; length=142; protocol=9; command=34; 
	*((uint32_t*)&(buf[8]))=off; // The ninth to twelfth byte of the message defines the offset of the data
	uint8_t*src=(uint8_t*)&(fdata[off]); // set a pointer to the offsetted position in the source array
	uint8_t*dst=(uint8_t*)&(buf[12]); // create a pointer to the thirteenth byte
	int pos=0;
	do{
		*dst++=*src++; // write the source to the destination
		pos+=1;
	}while(pos<128); 
}

static int loaddis(boost::asio::ip::tcp::socket* socket,int id){
	uint8_t rbuf[256],wbuf[256];
	uint64_t* wbuf64 = (uint64_t*)wbuf;
	int rlen,rps,rpl;
	fprintf(stderr,"Deactivating bootloader...");
	//  *((uint64_t*)wbuf)=0x1209800200ll|id;
	*(wbuf64) = 0x1209800200ll|id; // destination=id; source=2; flags=busallocation; protocol=9; command=18; 
	int result=wrr(socket,wbuf,0,rbuf,&rlen,&rps,&rpl); // send the message and try to receive a result
	if(result){ // if a reply was received
		if((rbuf[rps]==9)&&(rbuf[rps+1]==1)&&(rbuf[rps+2]==1)){// if the protocol is 9 and the command id is 1 and the first payload byte is also 1
			fprintf(stderr," ok\n");
			return 1;
		}else{
			fprintf(stderr," Unexpected answer.\n");
			ppack(&(rbuf[rps]),rpl);
			return 0;
		}
	}
	fprintf(stderr," No answer from target.\n");
	return 0;
}
// Run APPlication
static int rapp(boost::asio::ip::tcp::socket* socket, int id){
	uint8_t rbuf[256],wbuf[256];
	uint64_t* wbuf64 = (uint64_t*)wbuf;
	int retr,res,rlen,rps,rpl;
	for(;;){
		usleep(500000); // wait for 0.5 seconds
		fprintf(stderr,"Starting application...");
		*(wbuf64)=0x9800200ll|id; // destination=id; source=2; flags=busallocation; protocol=9; command=0; 
		wonce(socket,wbuf,0); // send the message
		usleep(500000); // wait for 0.5 seconds
		*(wbuf64)=0x1800200ll|id; // destination=id; source=2; flags=busallocation; protocol=1; command=0; 
		retr=5;
		for(;;){ // do the following five times maximally
			if(retr<=0){
				fprintf(stderr," Didn't get a device identity reply. The application firmware might be faulty.\n");
				return 0;
			}
			res=wrr(socket,wbuf,0,rbuf,&rlen,&rps,&rpl); // Send the message and try to get a reply
			if(res){ // if a reply was received
				if((rbuf[rps]==1)&&(rbuf[rps+1]==1)){ // if the protocol id and the command id are 1
					break;
				}else{
					ppack(&(rbuf[rps]),rpl); // print information about the received message
				}
			}else{
				fprintf(stderr,"?");
			}
			retr--;
		}
		if(rbuf[rps+17]&1){ // if the least significant bit of the eighteenth byte of the payload is "1", the bootloader started again (because the application did not start). If it is "0", the application started correctly.
			fprintf(stderr," Flashing failed or no valid firmware installed. The bootloader is running again.\n");return 0;
		}
		fprintf(stderr," Application is running.\n");
		break;
	}
	return 1;
}

static int sendcrc(boost::asio::ip::tcp::socket* socket,int id,uint8_t crc,int bsize){
	uint8_t rbuf[256],wbuf[256];
	uint64_t* wbuf64 = (uint64_t*)wbuf;
	uint32_t* wbuf32_8 = (uint32_t*)&(wbuf[8]);
	int rlen,rpl,rps;
	*(wbuf64)=0x2409000E900200ll|id|(((uint64_t)crc)<<56); // destination=id; source=2; flags=long and busallocation; protocol=9; command=36; 
	*(wbuf32_8)=bsize;
	fprintf(stderr,"Sending firmware CRC and size...");
	int res=wrr(socket,wbuf,0,rbuf,&rlen,&rps,&rpl); // send the message
	if(res){
		if((rbuf[rps]==9)&&(rbuf[rps+1]==1)&&(rbuf[rps+2]&&1)){ // if the protocol is 9 and the command id is 1 and the first payload byte is also 1
			fprintf(stderr," ok\n");
			usleep(20000); // wait for 0.02 seconds
			return 1;
		}else{
			fprintf(stderr," Unexpected answer.\n");
			ppack(&(rbuf[rps]),rpl); // print information about the received message
			return 0;
		}
	}
	fprintf(stderr," No answer from target.\n");
	return 0;
}

static int fdev(boost::asio::ip::tcp::socket* socket,int id,uint8_t*fdata,int bsize){
	uint8_t rbuf[256],wbuf[256];
	uint64_t* wbuf64 = (uint64_t*)wbuf;
	uint32_t* rbuf32_8 = (uint32_t*)&(rbuf[8]); // create a pointer to the ninth byte of the read buffer and interpret it as 32 bit unsigned integer
	int rlen,rpl,rps;
	fprintf(stderr,"Getting Flash memory information...");
	*(wbuf64)=0x209800200ll|id; // destination=id; source=2; flags=busallocation; protocol=9; command=2; 
	if(!wrr(socket,wbuf,0,rbuf,&rlen,&rps,&rpl)){ // send the message and try to receive an answer
		fprintf(stderr," No answer. Aborting.\n");return 0;
	}
	if((rbuf[rps]==9)&&(rbuf[rps+1]==1)&&(!rbuf[rps+2])){ // if the protocol is 9, the command id is 1 and the first payload byte is 0
		fprintf(stderr,"Critical error\n");
		*(wbuf64)=0x1009800200ll|id; // destination=id; source=2; flags=busallocation; protocol=9; command=16; 
		wonce(socket,wbuf,0); // send the message
		return 0;
	}
	if((rbuf[rps]!=9)||(rbuf[rps+1]!=3)){ // if the protocol is NOT 9 OR the command id is NOT 3 
		fprintf(stderr," Unexpected answer. Aborting.\n");
		ppack(&(rbuf[rps]),rpl); // print information about the message
		return 0;
	}
	int fps=rbuf[rps+2]<<6; // This seems to be the number of pages 
	int fp=rbuf[rps+3]|(rbuf[rps+4]<<8); // This seems to be the number of bytes per page
	int fs=fps*fp; // This is then the total number of bytes for all pages
	fprintf(stderr," %i pages of %i bytes (%i KiB).\n",fp,fps,fs>>10);
	fprintf(stderr,"Erasing application section in Flash memory...");
	*(wbuf64)=0x2009800200ll|id; // destination=id; source=2; flags=busallocation; protocol=9; command=32; 
	if(!wrr(socket,wbuf,0,rbuf,&rlen,&rps,&rpl)){ // send the message and try to receive an answer
		fprintf(stderr," No answer. Aborting.\n");return 0;
	}
	if((rbuf[rps]==9)&&(rbuf[rps+1]==1)&&(!rbuf[rps+2])){ // if the protocol is 9, the command id is 1 and the first payload byte is 0
		fprintf(stderr," Critical error\n");
		*(wbuf64)=0x1009800200ll|id; // destination=id; source=2; flags=busallocation; protocol=9; command=16; 
		wonce(socket,wbuf,0); // send the message
		return 0;
	}
	if((rbuf[rps]!=9)||(rbuf[rps+1]!=1)||(rbuf[rps+2]!=1)){ // if a message was received whose protocol is not 9, whose command id is not 1 and whose first payload byte is not 1
		fprintf(stderr," Unexpected answer. Aborting.\n");
		ppack(&(rbuf[rps]), rpl);
		return 0;
	}
	fprintf(stderr," ok\n");
	usleep(6000); // wait for 0.06 seconds
	fprintf(stderr,"Sending application firmware data");
	int fdone=0;
	do{*(wbuf64)=0x409800200ll|id; // destination=id; source=2; flags=busallocation; protocol=9; command=4; 
		if(!wrr(socket,wbuf,0,rbuf,&rlen,&rps,&rpl)){ // write the message and try to receive an answer
			fprintf(stderr," Timeout.\n");return 0;
		}
		if(rbuf[rps]!=9){ // if the anwer's protocol id is not 9
			fprintf(stderr," Failed.\nUnexpected reply.\n");
			ppack(&(rbuf[rps]),rpl);
			return 0;
		}
		switch(rbuf[rps+1]){ //switch the command id
			case 0x01: // if the command id is 1
				switch(rbuf[rps+2]){ // if the first payload byte ..
					case 0x02: // is 2
						fprintf(stderr,"d"); // no idea what that could mean
						usleep(1000); // wait for 0.001 seconds
						break;
					default:
						fprintf(stderr," Critical error\n");
						return 0;
					}
				break;
			case 0x21:{ // if the command id is 33
				int32_t off=*(rbuf32_8)&0xFFFFFF;// the ninth byte of the message (interpreted as 32 bit unsigned int) seems to contain the offset of the received data (microcontroller-wise). This offset is then modified such that the least significant 24 bits are set to 1. The offset must therefore be a multiple of 16.777.215.
				if(off==bsize){ // if the offset equals the size of the write buffer, the task is completed.
					fdone=1;
					break;
				}else if(off>bsize){ // if the offset is bigger than the buffer size, something obviously went wrong.
					fprintf(stderr," Critical error\n");return 0;
				}
				pdatpak(wbuf,id,fdata,off); // create a message that contains firmware data
				wonce(socket,wbuf,0);// send that message
				fprintf(stderr,".");
				break;
			}
			default: // if the command id is something else
			fprintf(stderr," Critical error\n");return 0;
		}	
	}while(!fdone);
	fprintf(stderr," done.\n");
	return 1;
}

/*
 * "FlexLoader (c) 2010-2013 Mattias Schaeffersmann\n"
	"Usage:\n"
	"  flexfirmware <id> <port> -f <file.hex>\n"
  "    Load firmware from file.hex into device flash memory.\n"
  "  flexfirmware <id> <port> -r\n"
  "    Run current application and leave bootloader.\n"
  "  Device ID can be decimal (16) or hexadecimal (0x10).\n"
#if defined __CYGWIN__
  "  Serial port number should be the Window COM port number.\n"
  "  It will be prepended with /dev/ttyS for Cygwin.\n"
#elif defined __linux__
  "  Serial port number will be prepended with /dev/ttyACM.\n"
#endif
*/


int flexloader(boost::asio::ip::tcp::socket* socket, unsigned char id, const std::string hexfile) {
	int option = -1;
	static int bsize=0;
	int i;
	static std::vector<unsigned char> fdata;
	static uint8_t fcrc=-1;
	
	//char*pptr=NULL;
	//id=strtol(argv[1],&pptr,0);
	//if(*pptr){
	//fprintf(stderr,"Error while parsing device ID.\n");return 1;
	//}
	//port=strtol(argv[2],&pptr,0);
	//if(*pptr){
	//fprintf(stderr,"Error while parsing port number.\n");return 1;
	//}
	if (hexfile.size()!=0) option^=option;//equals option=0, but much more nerdy.
	else option=1; 
	//if(!strcmp(argv[3],"-f")) option^=option;
	//else if(!strcmp(argv[3],"-r")) option=1;
	//if(option==-1){
	//fprintf(stderr,"Unknown option.\n");return 1;
	//}
	static std::string lastHexfile="";
	if(!option && lastHexfile!=hexfile){ //so, if a hexfile was provided
		lastHexfile=hexfile;
		fdata.clear();
		fcrc=-1;
		bsize=0;
		//if(argc<5){
		//	fprintf(stderr,"Missing firmware filename.\n");return 1;
		//}
		char tmp2[]="flfw_XXXXXX";//Let's assume that this is supposed to be a filename.
		char *tmp=mktemp(tmp2);//Then this command will make a unique filename out of it.
		fprintf(stderr,"Converting Intel hex file to binary...\n");
		auto fidTemp=fopen(hexfile.c_str(),"r"); // Try to open the file in order to see if it really exists
		fclose(fidTemp);// If the program didn't crash, the file seems to exist.
		int pid=fork(); // This creates a copy of the current process
		if(!pid){ // Probably, if this is the child process (pid==0), this will be executed.
			execlp("avr-objcopy","avr-objcopy","-I","ihex","-O","binary",hexfile.c_str(),tmp,(void*)0); //execlp starts a program whose path is defined by the first parameter. The following parameters are passed as attributes. The first argument, however, by convention is set to be the name of the program to be started. 
			// avr-objcopy "copies and translates object files".
			// "-I"/"--input-target=" tells the program what the input object format is rather than let it decide this on its own.
			// "-O"/"--output-target=" tells the program what the output format should be.
			// It seems that an object file that was saved in intel hex format is to be converted into a binary.
			// Therefore, a new file is created and the converted data from the source file is written to it.
			execlp("objcopy","objcopy","-I","ihex","-O","binary",hexfile.c_str(),tmp,(void*)0); // Same as above. Only, if the first call of execlp fails
			fprintf(stderr,"Critical convert error\n"); // This line should not be reached if one of the above calls worked out.
			exit(42); // This is the error number for the case that the above calls didn't work.
		}
		int status=-1;
		wait(&status); // I assume that this waits for the child processes to finish.
		if(!WIFEXITED(status)){ // WIFEXITED returns zero if a process terminated normally. Therefore, this checks whether anything went wrong.
			fprintf(stderr,"Critical error\n");
			return 1; // If something went wrong, the program ends.
		}
		if(WEXITSTATUS(status)){ //WEXITSTATUS returns the least significant eight bits of the exit status of the child process. Therefore, this will only be called if the child process terminated abnormally. Since the program should have already exited in the previous if-statement if anything went wrong, it is not clear, under which circumstances this if can be reached.
			if(WEXITSTATUS(status)!=42){ // If the child process terminated with an error status other than 42, the program ends after the print. It is not clear 
				fprintf(stderr,"avr-objcopy failed\n");
				return 1;
			}
		}
		struct stat fst; // This creates a status struct based on the definition given in sys/stat.h. 
		if(stat(tmp,&fst)){ // The stat()-function is supposed to optain information about the file name that is passed as the first attribute and write this information to the area pointed to by the second argument. If the function succeeds, 0 will be returned; otherwise it returns -1. 
			fprintf(stderr,"Critical error\n");
			unlink(tmp); // Delete the name and the file it refers to.
			return 1;
		}
		int sreal=fst.st_size;// read the total size (in bytes)
		bsize=(sreal+511)&~(511);// This rounds sreal to the next fill multiple of 512: 0->0; 1->512; 512->512; 513->1024 ...
		fdata.resize(bsize); // Allocate dynamic memory.
		int fid=open(tmp,O_RDONLY); // Open the file
		int iores=read(fid,fdata.data(),sreal); // read the whole file into the allocated memory
		if(iores!=sreal){ // If the number of bytes that was read does not equal the number of bytes that should have been read...
			fprintf(stderr,"Critical error\n"); // something went wrong.
			unlink(tmp);
			return 1;
		}
		close(fid); // close the file
		unlink(tmp); // delete it
		for(i=sreal;i<bsize;i++){ // fill the remaining bytes of the allocated memory with 0xFFs
			fdata[i]=0xFF; 
		};
		// Now the crc of the data is computed.
		fcrc=0xFF;
		for(i=0;i<bsize;i++){
			int j;
			fcrc=fcrc^fdata[i];
			for(j=0;j<8;j++){
				if(fcrc&0x01){
					fcrc=(fcrc>>1)^0x8C;
				}else{
					fcrc>>=1;
				}
			}
		}
		fprintf(stderr,"Successfully read %i bytes of firmware data. CRC=%X\n",sreal,fcrc);
	}
	// fprintf(stderr,"Opening serial port %i to communicate with the bus master...\n",port);
	//char dname[16]; // The name of the port
	uint8_t wbuf[256],rbuf[256];
	uint64_t* wbuf64 = (uint64_t*)wbuf;
	int rlen,rps,rpl,retr,res;

	fprintf(stderr,"ok\n");
	rps=rpl=0;


	usleep(500);
	fprintf(stderr,"Querying target device 0x%X...",id);
	for(;;){
		*(wbuf64)=0x1800200ll|id; // destination=id; source=2; flags=busallocation; protocol=1; command=0; payload={0,0}; This message asks the client to send some identification data.
		retr=5;		// maximum number of attempts
		for(;;){
			if(retr<=0){
				fprintf(stderr," Didn't get a device identity reply. Is the target device connected to the bus?\n");return 1;
			}
			res=wrr(socket,wbuf,0,rbuf,&rlen,&rps,&rpl); // Send the message and try to get a response
			if(res){ // if a reply was received
				if((rbuf[rps]==1)&&(rbuf[rps+1]==1)){ // if the first two bytes of the payload (protocol and command) are both 1, break. This is the correct response for the identity request.
					break;
				}else{
					ppack(&(rbuf[rps]),rpl);
				};
			}else{ // if no reply was received
				fprintf(stderr,"t");
			}
			retr--;
		}
		if(rbuf[rps+17]&1){//if the least significant bit of the the 18. byte of the payload is 1. What does that mean?
			fprintf(stderr," Got response from bootloader.\n");
			break;
		}
		if(!option){ // If the application hex-file shall be transfered
			fprintf(stderr," Application running.\nRequesting restart into bootloader...");
			*(wbuf64)=0x1009800200ll|id; // destination=id; source=2; flags=busallocation; protocol=9; command=16; payload={0,0}
			wonce(socket,wbuf,0);
			usleep(16000);
		}else if(option==1){ // The bootloader should exit in order to run the application
			fprintf(stderr," Target device is already running an application.\nDone.\n");
			return 0;
		}
	}
	if(!option){ // If the application hex-file should be transfered
		if(!fdev(socket,id,fdata.data(),bsize)){
			fprintf(stderr,"Device flashing failed. Aborting.\n");
			return 1;
		}
		sendcrc(socket,id,fcrc,bsize);
	}
	if(!rapp(socket,id)){
		fprintf(stderr,"Application start failed. Aborting.\n");
		return 1;
	}
	if (!loaddis(socket,id)){
		fprintf(stderr,"Failed to deactivate bootloader. Is the firmware alright?\n");
	}
	//close(socket);
	fprintf(stderr,"Done.\n");
	return 0;
}