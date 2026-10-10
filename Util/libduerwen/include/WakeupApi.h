#ifndef _WAKEUP_API_H_
#define _WAKEUP_API_H_

#include <stdio.h>
#include <stdlib.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef void *HWWakeup;
/*
params:
HwWakeuphandle:
filename:
return :0 
*/
extern int Duerwen_wakeup_init(HWWakeup *handle,int lang);

/*
params:
HwWakeuphandle:
mic1_data:sample short 
mic2_data:sample short
mic3_data:sample short  
len:1024
return :0 
timie :1 hours
*/
extern int Duerwen_wakeup_three_write_data(HWWakeup handle,short *mic1_data,short *mic2_data,short *mic3_data,short *ref_data,short *outdata1,short *outdata2,short *outdata3,short *naec_data);
/*
*params:
*handle
*Iswakeup:命令词反馈
*mic1_data
*mic2_data
*mic3_data
*len:1024
* return: 角度0~360
*/
extern int Duerwen_doa_three_write_data(HWWakeup handle,int *Iswakeup,short *mic1_data,short *mic2_data,short *mic3_data,short *naec_data,int len)
;
/*
params:
HwWakeuphandle:
return :0 
*/
extern int Duerwen_wakeup_unit(HWWakeup handle);

#ifdef __cplusplus
}
#endif


/*
* example demo

static int tm=0;
int main(){
	HWWakeup Handle00;
	Duerwen_wakeup_init(&Handle00,0);
	short MIC1[1024];
	short MIC2[1024];
	short MIC3[1024];
	short MIC[1024*3];
	short ref[1024];
	short aec1_out[1024];
	short aec2_out[1024];
	short aec3_out[1024];
	FILE *fre_in = fopen("./ysc.wav","rb");
	int ret = fread(ref, 1,44,fre_in);
    int i =0;
	while (1)
	{
		ret = fread(MIC, 2,1024*3,fre_in);
		if(ret != 1024*3) break;
   		for(i=0;i<1024;i++){
   			MIC1[i] = MIC[6*i];
   			MIC2[i] = MIC[6*i+1];
   			MIC3[i] = MIC[6*i+2];  
   			ref[i] = MIC[6*i+4];   			
   		}

		Duerwen_wakeup_three_write_data(Handle00,MIC1,MIC2,MIC3,ref,aec1_out,aec2_out,aec3_out);
		ret = Duerwen_doa_three_write_data(Handle00,aec1_out,aec2_out,aec3_out,1024);
		if(ret >0){
			printf("--------->is_wakeup:%d\n",ret);
		}
	}
	Duerwen_wakeup_unit(Handle00);
	return 0;
}
*/





#endif
