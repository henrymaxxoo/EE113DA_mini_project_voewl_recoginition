#include <stdio.h>
#include "L138_LCDK_aic3106_init.h"
#include "L138_LCDK_switch_led.h"
#include<cmath>
#include "evmomapl138_gpio.h"
#include <stdint.h>
#include <math.h>
#include <ti/dsplib/dsplib.h>



////////////////////////////////////////////////////////////////global variable//////////////////////////////////////////////////////////
#define N 1024
#define PI 3.1415926
int switch_state = 0;
int flag = 0;
int count = 0;
int recording[8000];
int sound_print[1024];
float hamming_window[1024];

int index = 0;
int sound;
float reverb;
char flag2 = 'T';
float PSD[513];

int count_6=0, count_7=0, count_8=0,count_flag=0;
float x1_offset[13] = {2.213154,-11.89834,-5.080175,-12.99322,-14.44497,-10.18346,-7.471984,-8.494889,-7.055241,-8.570406,-10.09429,-6.72916,-7.327752};
float x1_step1_gain[13] = {0.125255552641276,0.113124200247256,0.138857723044383,0.108340976789246,0.0947273972851886,0.11320047107244,0.1462406607972,0.152953107712714,0.125371907929128,0.116033665311451,0.108532016565025,0.144226002145362,0.158566495454493};
float x1_step1_ymin = -1.00;
float b1[5]= {-1.2857994110127834997,-0.65521699678469791372,0.053037793975103900146,-0.83027841449504480309,-1.062301544878340831};
float b2[4]= {-0.027094535104604119957,-0.10104868111011418297,0.51642077775276495899,-0.60600684226324141868};
float LW_2 [4][5]  = {{0.91223261827321655559, -0.85711570277027560039, 1.5073812734169040883, -0.47889220264949006189, -0.17529485686149942625},
                    {-0.40363212553834781904, -1.640909815105186853, -0.027447654600533666347, -0.86366885102684165521, 1.37621024838710726},
                      {0.49269223429725728103, 1.4680569895060913144,1.0786458313549294186, 0.86364687983031163121, 0.64465879490708999366},
                      {0.082680858853868360869, -0.21300401726134099434, -1.6816401339119526614, -0.48935913626367633089, -0.75488571395398684771}};

float LW_1 [5][13] = {{0.73139600480160604778, 0.41171313961460875852, 0.18168449617714793609, -0.45154081742269058797, -0.052909897927440163734, -0.88815160062314413292, 0.78010334892325461364, -0.62609200058289615765, 0.30595710637955203515, 0.6324194701165213317, 0.21153693491462510767, 0.04147864511470015364, -0.2465667620452209885},
                      {1.1721488570938303209, 0.41069192883338151434, -0.18392593990959194006, 0.56865865050645880707, 0.82400078913644170253, -0.34998393620756829137, -0.44612465089535413565, 0.86853571633104542649, -0.062775310599748446183, 0.042696430861228221176, 0.89066902052542984425, -1.0361382037168846981, -0.19162133423777913399},
                      {0.33911386761173067939, 0.9974361801079377754, -0.94212917326825329489, -0.86151623105822960902, 0.62158334394535941225, -0.13829925747765967059, 0.92399849111069909391, -0.049080594534330841683, 0.73334403728530761502, 0.42149289201184891862, 0.70134895029719312998, 0.1191055726464085307, -0.032826621437783649882},
                      {1.0886636311645856434, -0.028613300018244393341, -0.30755331027641324981, 0.23150932029670170942, 1.040497374843816436, 0.52386880771370047949, 0.052865768854650815878, 0.093354850654274915134, -0.52181836967955175233, -0.18708766759114092704, -0.23485361177573896296, -0.54922341253383344384, 0.21745112712546205147},
                      {-0.33679787685877904391, 0.62326229661772403201, 1.0886778912770860295, -0.33997545572462245511, 0.18229624195706001233, 0.047428583602953267029, -0.35619682416039749606, 0.59075343953610404135, 0.84822207931905169165, -0.95027662686069258591, 0.082056463355188818953, 0.80369653697774512402, -0.044461329832392176842}};
float Y[13];
float h[5];
float o[4];
float weight_total=0.00;
float weight_out=0.00;
float o_SUM = 0.00;
float s=0.00;
float final_output[4];

float check[4], check_1[5];

unsigned char brev[64] = {
    0x0, 0x20, 0x10, 0x30, 0x8, 0x28, 0x18, 0x38,
    0x4, 0x24, 0x14, 0x34, 0xc, 0x2c, 0x1c, 0x3c,
    0x2, 0x22, 0x12, 0x32, 0xa, 0x2a, 0x1a, 0x3a,
    0x6, 0x26, 0x16, 0x36, 0xe, 0x2e, 0x1e, 0x3e,
    0x1, 0x21, 0x11, 0x31, 0x9, 0x29, 0x19, 0x39,
    0x5, 0x25, 0x15, 0x35, 0xd, 0x2d, 0x1d, 0x3d,
    0x3, 0x23, 0x13, 0x33, 0xb, 0x2b, 0x1b, 0x3b,
    0x7, 0x27, 0x17, 0x37, 0xf, 0x2f, 0x1f, 0x3f
};
float H[26];
int lamda[28];
float x_sp[2048];
float y_sp[2048];
float w_sp[2048];
float y_real_sp [N];
float y_imag_sp [N];
float magnitute[N];
float x_m[26];
int a = 0;
int j=0;
int K=513, M=26,F=8000;
int m,k;
float fai[26];
float MFCC[13];
float fmel0;
   float fme_M;
   float gap;

////////////////////////////////////////////////////////////////global variable/end//////////////////////////////////////////////////////
void lightoff(){
	count=0;
	index=0;
	LCDK_LED_off(4);
   LCDK_LED_off(5);
	LCDK_LED_off(6);
    LCDK_LED_off(7);
}
void gen_twiddle_fft_sp (float *w, int n)
{
    int i, j, k;
    double x_t, y_t, theta1, theta2, theta3;

    for (j = 1, k = 0; j <= n >> 2; j = j << 2)
    {
        for (i = 0; i < n >> 2; i += j)
        {
            theta1 = 2 * PI * i / n;
            x_t = cos (theta1);
            y_t = sin (theta1);
            w[k] = (float) x_t;
            w[k + 1] = (float) y_t;

            theta2 = 4 * PI * i / n;
            x_t = cos (theta2);
            y_t = sin (theta2);
            w[k + 2] = (float) x_t;
            w[k + 3] = (float) y_t;

            theta3 = 6 * PI * i / n;
            x_t = cos (theta3);
            y_t = sin (theta3);
            w[k + 4] = (float) x_t;
            w[k + 5] = (float) y_t;
            k += 6;
        }
    }
}

separateRealImg () {
    int i, j;

    for (i = 0, j = 0; j < N; i+=2, j++) {
        y_real_sp[j] = y_sp[i];
        y_imag_sp[j] = y_sp[i + 1];
        magnitute[j] = (float) sqrt(pow(y_real_sp[j],2) + pow(y_imag_sp[j],2));
    }

    for(j=0;j<513;j++){
    	   PSD[j]=(float)(pow(y_real_sp[j],2) + pow(y_imag_sp[j],2));
    }
}

interrupt void interrupt4(void)  // interrupt service routine
{

  flag = LCDK_SWITCH_state(5)+LCDK_SWITCH_state(6)+LCDK_SWITCH_state(7)+LCDK_SWITCH_state(8);

  //////////////////////////////////switch 5//////////////////////////////////////
  if(LCDK_SWITCH_state(5)) {
	  if(count < 8000)   LCDK_LED_on(4);
	  else if (count<8000*2){
	  LCDK_LED_off(4);
	  LCDK_LED_on(5);
	 }
	  else if(count<8000*3){
	  LCDK_LED_off(5);
      LCDK_LED_on(6);
	  }

	  else if(count<8000*4){
      LCDK_LED_off(6);
	  LCDK_LED_on(7);
	  sound = input_left_sample();
	  recording[index++]=sound;
	  }

	  if(index >= 5 && index <= 1029){
	 sound_print[index - 5] = recording[index];
	 hamming_window[index-5]=(0.54-0.46*cos((2.00*3.14*(index-5))/1023.00))*(double)recording[index];
     }

	  else if(index > 1029)
	  {
	 LCDK_LED_off(7);
	  flag2='F';
	  }

	  count++;

  }

///////////////////////////////////end of switch//////////////////////////////////////

  if (flag==0) {
 	  lightoff();
   }

if(flag>1){
	reverb=0;
	if(count_flag<4000){
			LCDK_LED_on(4);
			LCDK_LED_on(5);
		    LCDK_LED_on(6);
			LCDK_LED_on(7);
	}
		else if(count_flag>4000 && count_flag<8000){
			//lightoff();
		}

	else if(count_flag>8000)
		count_flag=0;

	count_flag++;
}

  output_left_sample(reverb);
  return;

}

int main(void)
{

   L138_initialise_intr(FS_16000_HZ,ADC_GAIN_24DB,DAC_ATTEN_0DB,LCDK_MIC_INPUT);
   LCDK_LED_init();
   LCDK_SWITCH_init();
   LCDK_GPIO_init();


while(1)
 {
	if(flag2=='F')
  {


   for(j=0; j<1024;j++)
   {
        x_sp[2*j]=hamming_window[j];
        x_sp[2*j+1]=(float)0.00;
   }

   gen_twiddle_fft_sp(w_sp,N);
   DSPF_sp_fftSPxSP(N,x_sp,w_sp,y_sp,brev,4,0,N);
   separateRealImg ();


   fmel0=2595*log10(1+(250.00/700.00));
   fme_M=2595*log10(1+(8000.00/700.00));
   gap = (fme_M-fmel0)/27;

   for( m=0;m<M;m++){
       fai[m]=(float)700*(pow(10,(fmel0+gap*(m+1))/2595)-1);
   }

   for(m=0;m<28;m++){
	   if(m==0) lamda[m]=(int) (K*250/8000);
	   else if(m==27) lamda[m]=(int)(K*8000/8000);
	   else
		   lamda[m]=(int) (K*fai[m-1]/8000);
   }

   for(m=1;m<=M;m++){
	   float temp=0.00;
	   for(k=0;k<K;k++){

		   if(k<=lamda[m-1]){
			   temp+=0.00;
		   }

		   else if(lamda[m-1]<k && k<lamda[m]){
			   temp+=(float)((k-lamda[m-1])/(lamda[m]-lamda[m-1]))*(float)PSD[k];
		   }

		   else if(lamda[m]<=k && k<lamda[m+1]){
			   temp+=(float)((lamda[m+1]-k)/(lamda[m+1]-lamda[m]))*(float)PSD[k];
		   }

		   else if(lamda[m+1]<=k){
			   temp+=0.00;
		   }
	   }

	   H[m-1]=temp;
   }

   for(m=0;m<M;m++){
	   x_m[m]= (float) log10(H[m]);
   }

   int z;
   for (z=1;z<=13;z++){
	   float sum = 0.0;
	   for(m=0;m<M;m++){
		   float a=PI/26;
		   sum+= x_m[m]*cos(z*(m+1-0.5)*a);
	   }
	   MFCC[z-1]=sum;
   }

   ////////////////////////neuron study//////////////////////////////////////////////////////////////
   ///////////////////////////////Neron Network////////////////////////////////////////
      for(z=0;z<13;z++){
        Y[z]=MFCC[z] - x1_offset[z];
        Y[z]=Y[z]*x1_step1_gain[z];
        Y[z]=Y[z]+x1_step1_ymin;
      }

      int x,k;


     for (x=0; x<5;x++){
           s= 2/(1+exp(-2*(x+1))) -1;
       for(k=0;k<13;k++){
           weight_total+= LW_1[x][k]*Y[k];
       }
       check_1[x]=weight_total;
       h[x]= s*(weight_total+b1[x]);
       weight_total=0.00;
     }

     for (x=0;x<4;x++){
       for(k=0;k<5;k++){
           weight_out+=LW_2[x][k]*h[k];
       }
       check[x]=weight_out;
       o[x]=weight_out+b2[x];
       weight_out=0.00;
     }

     /*for (k=0;k<4;k++){
       o_SUM+= exp(o[k]);
     }*/
     o_SUM= exp(o[0])+exp(o[1])+exp(o[2])+exp(o[3]);

     for(x=0;x<4;x++){
       final_output[x]=exp(o[x])/o_SUM;
     }
     ///////////////////////////////////////study end///////////////////////////
   flag='T';
  }

 }
   while(1)
   {}
}
