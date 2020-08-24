
/*//----------------------------------------------------
#define FACE_ID_SIZE 512
change update in fr_forward.h  
//----------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include "esp_camera.h"
#define CAMERA_MODEL_AI_THINKER
#include <WiFi.h>
#include "soc/soc.h" //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems
#include "camera_pins.h"

#include "face.h"

#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "Arduino.h"

#include "fb_gfx.h"
#include "fd_forward.h"
#include "fr_forward.h"

#include "FS.h"
#include "SPI.h"
#include "SPIFFS.h"
#include "SD.h"
#include "sd_diskio.h"
#include <Arduino_JSON.h>

#include <TFT_eSPI.h>       // Hardware-specific library
TFT_eSPI tft = TFT_eSPI();  // Invoke custom library

const char* ssid     = "fortuna";
const char* password = "fipl@123";

const char* RemoteIP = "192.168.0.25";
const char* RemotePort = "4007";

unsigned char ch = 0;
int connect_flag = 0;
WiFiClient client;
JSONVar myObject ;
int conn_flag = 0;

unsigned int integer = 0;
unsigned int disconn = 0;


#define FACE_COLOR_WHITE  0x00FFFFFF
#define FACE_COLOR_BLACK  0x00000000
#define FACE_COLOR_RED    0x000000FF
#define FACE_COLOR_GREEN  0x0000FF00
#define FACE_COLOR_BLUE   0x00FF0000
#define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
#define FACE_COLOR_CYAN   (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
#define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)

//--------------------------------------------------------------------------------
#define FORMAT_SPIFFS_IF_FAILED true
void deleteF(fs::FS &fs, const char * path);
void readF(fs::FS &fs, const char * path);
void appendF(fs::FS &fs, const char * path, const char * message);
void writeF(fs::FS &fs, const char * path, const char * message);
void readSFace(fs::FS &fs, const char * path, int lineNum);
//--------------------------------------------------------------------------------
# define STORAGE_DEVICE SPIFFS  //SPIFFS SD
#define USER_ENROLL_COUNT     (100*1) //

#define VECTOR_ARRAY_SIZE     512 // as per 1MB vfs flash
#define USER_ID_VECTOR_SIZE   128  //128 node point to be calculate
#define VECTOR_BLOCK_SIZE     (VECTOR_ARRAY_SIZE/USER_ID_VECTOR_SIZE) //   bundle size
#define USER_INFO             32  // as per 1MB vfs flash   
#define FACE_REC_THRESHOLD    .80
#define ENROLL_CONFIRM_TIMES  1
#define D_BUF_LEN             ((VECTOR_ARRAY_SIZE)*6) 

static char  fac_buf[D_BUF_LEN]={0};
static char  s[10] = {0};
static int   face_ram_buf_count = 0;
static float face_ram_buf[USER_ENROLL_COUNT][USER_ID_VECTOR_SIZE] ={0};
static char  SD_tmp_buf[512] ="";
static char  tmp_buf_1[3024] ="";

typedef struct {
        size_t size; //number of values used for filtering
        size_t index; //current value index
        size_t count; //value count
        int sum;
        int * values; //array to be filled with values
} ra_filter_t;

ra_filter_t ra_filter;
mtmn_config_t mtmn_config = {0};
face_id_list id_list = {0};

char chh = 0;
int8_t detection_enabled = 1;
int8_t recognition_enabled = 1;
int enroll_enable = 0; 
int scan_enable = 0;
int sample_face_count = 0;

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap);
ra_filter_t * ra_filter_init(ra_filter_t * filter, size_t sample_size);
esp_err_t face_detection (void);
int run_face_recog(dl_matrix3du_t *image_matrix, box_array_t *net_boxes);
int8_t recog_fc(dl_matrix3du_t *algined_face);
int rgb_printf(dl_matrix3du_t *image_matrix, uint32_t color, const char *format, ...);
static void draw_face_box(dl_matrix3du_t *image_matrix, box_array_t *boxes, int face_id);
static void rgb_print(dl_matrix3du_t *image_matrix, uint32_t color, const char * str);

void testFileIO(fs::FS &fs, const char * path);
void deleteFile(fs::FS &fs, const char * path);
void renameFile(fs::FS &fs, const char * path1, const char * path2);
void appendFile(fs::FS &fs, const char * path, const char * message);
void writeFile(fs::FS &fs, const char * path, const char * message);
void readFile(fs::FS &fs, const char * path);
void removeDir(fs::FS &fs, const char * path);
void createDir(fs::FS &fs, const char * path);
void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
//====================================================================================
//====================================================================================

void DisplayMatrixImage(uint16_t w,uint16_t h,uint8_t *item ){
  
    unsigned long  i = 0,j = 0,k = 0, pixel = 0,val = 0;
    uint8_t *s = (uint8_t*)item;
    uint16_t ww,*d = (uint16_t*)s;
    uint16_t r = 0,g = 0,b = 0;
    k = pixel = w * h ;
    do {
        //ww  = (*s++ & 0xF8) << 8;   // RRRRR-----------  
        //ww |= (*s++ & 0xFC) << 3;   // -----GGGGGG-----  
        //ww |= (*s++  ) >> 3;        // -----------BBBBB  

        r = (uint16_t)(*s++ & 0xF8); 
        g = (uint16_t)(*s++ & 0xFC);
        b = (uint16_t)(*s++ & 0xF8);  
        ww  = b << 8;
        ww |= g << 3;
        ww |= r >> 3;
        
        *d++ = ww;
    } while (--pixel); 

    uint8_t  *ss = (uint8_t*)item;
    uint16_t *e  = (uint16_t*)ss;
    uint16_t *f  = (uint16_t*)ss;


    for(int i=0;i<240;i++){
        for(int j=0;j<320;j++){
          
            if(j>39 && j<240+40){   *e++ = *f++;  }
            else{  *f++;  }
        }
   }
    
    tft.setAddrWindow(0, 0,240,240);  //tft.setAddrWindow(0, 0,w,h);  
    tft.setSwapBytes(1);
    tft.pushColors((uint8_t*)item, (240*240)*2); // With byte swap option

    //listDir(SD, "/", 0);
}

//===============================================================================================================//
//===============================================================================================================//


void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.name(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

void testFileIO(fs::FS &fs, const char * path){
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if(file){
        len = file.size();
        size_t flen = len;
        start = millis();
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms\n", flen, end);
        file.close();
    } else {
        Serial.println("Failed to open file for reading");
    }


    file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for(i=0; i<2048; i++){
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
}

//===============================================================================================================//
//===============================================================================================================//
ra_filter_t * ra_filter_init(ra_filter_t * filter, size_t sample_size)
{
    memset(filter, 0, sizeof(ra_filter_t));

    filter->values = (int *)malloc(sample_size * sizeof(int));
    if(!filter->values){
        return NULL;
    }
    memset(filter->values, 0, sample_size * sizeof(int));

    filter->size = sample_size;
    return filter;
}
//===============================================================================================================//
void deleteF(fs::FS &fs, const char * path){
    if(!fs.remove(path)){   Serial.println("- delete failed");    } 
}

void writeF(fs::FS &fs, const char * path, const char * message)
{
    File file = fs.open(path, FILE_WRITE);
    if(!file){     return;   }
    if(!file.print(message)){     Serial.println("- write failed");   } 
}

void readF(fs::FS &fs, const char * path)
{
    char ch = 0;
    File file = fs.open(path);
    if(!file || file.isDirectory()){     return;   }
    while(file.available()){
        ch = file.read();  
        Serial.write(ch);
    }
}

void appendF(fs::FS &fs, const char * path, const char * message)
{
    File file = fs.open(path, FILE_APPEND);
    if(!file){   return;   }
    if(!file.print(message)){     Serial.println("- append failed");   } 
}
//===============================================================================================================//



void setup() {

    //-------------------------------------------------------------------------------------//
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
    Serial.begin(115200);
    Serial.printf("\n\nGetFreeHeap: ");
    Serial.println(ESP.getFreeHeap());
    //-------------------------------------------------------------------------------------//
    //-------------------------------------------------------------------------------------//
    IPAddress local_IP(192, 168, 0, 160);
    IPAddress gateway(192, 168, 0, 11);
    IPAddress subnet(255, 255, 255, 0);
    IPAddress primaryDNS(192, 168, 0, 11); //optional
    IPAddress secondaryDNS(8, 8, 4, 4); //optional
    
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
      Serial.println("STA Failed to configure");
    }
    
    Serial.print("Connecting to ");
    Serial.println(ssid);
    
    WiFi.begin(ssid, password);
    
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }

    Serial.printf("\n WiFi connected: ");
    Serial.print(WiFi.localIP());

    delay(1000); 
   
    //-------------------------------------------------------------------------------------//
    //-------------------------------------------------------------------------------------//
    if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
       Serial.println("SPIFFS Mount Failed");
        return;
    }
    //-------------------------------------------------------------------------------------//
    //-------------------------------------------------------------------------------------//   
    tft.init();
    tft.setRotation(2);
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0, 4);      // Set "cursor" at top left corner of display (0,0) and select font 4
    tft.setTextColor(TFT_RED, TFT_BLACK); // Set the font colour to be white with a black background
    tft.println("\n   FALCON 11 \n"); // We can now plot text on screen using the "print" class
    //tft.println("White text");
    tft.setTextColor(TFT_BLUE, TFT_BLACK);
    tft.println("\n   FIPL Face IT\n");
    
    delay(1000);
    //-------------------------------------------------------------------------------------//
    //-------------------------------------------------------------------------------------//
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    //init with high specs to pre-allocate larger buffers
    if(psramFound()){
      config.frame_size = FRAMESIZE_UXGA; // 1600x1200
      config.jpeg_quality = 10;
      config.fb_count = 2;
    } else {
      config.frame_size = FRAMESIZE_SVGA; // 800x600
      config.jpeg_quality = 12;
      config.fb_count = 1;
    }
 
    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {   return;  }
    
    sensor_t * s = esp_camera_sensor_get();
    s->set_framesize(s, FRAMESIZE_QVGA); //drop down frame size for higher initial frame rate
    
    //initial sensors are flipped vertically and colors are a bit saturated
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);
    s->set_special_effect(s, 2);  //2 gray   0 No Effect
    //-------------------------------------------------------------------------------------//

    mtmn_config.type = FAST;
    mtmn_config.min_face = 80;
    mtmn_config.pyramid = 0.707;
    mtmn_config.pyramid_times = 4;
    mtmn_config.p_threshold.score = 0.6;
    mtmn_config.p_threshold.nms = 0.7;
    mtmn_config.p_threshold.candidate_number = 20;
    mtmn_config.r_threshold.score = 0.7;
    mtmn_config.r_threshold.nms = 0.7;
    mtmn_config.r_threshold.candidate_number = 10;
    mtmn_config.o_threshold.score = 0.7;
    mtmn_config.o_threshold.nms = 0.7;
    mtmn_config.o_threshold.candidate_number = 1;
    
    ra_filter_init(&ra_filter, 20);
    face_id_init(&id_list, 1, ENROLL_CONFIRM_TIMES); // FACE_ID_SAVE_NUMBER = 1
    //-------------------------------------------------------------------------------------//
    //-------------------------------------------------------------------------------------//

}

camera_fb_t* capture(){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    fb = esp_camera_fb_get();
    return fb;
}

void showImage(){
    camera_fb_t *fb = capture();
    if(!fb || fb->format != PIXFORMAT_JPEG){
      Serial.println("Camera capture failed..");
      esp_camera_fb_return(fb);
      return;
    }else{
      esp_camera_fb_return(fb);
    }
}

void loop() {
    //showImage();

    face_detection();









    
}
//===============================================================================================================//
//===============================================================================================================//
static void rgb_print(dl_matrix3du_t *image_matrix, uint32_t color, const char * str)
{
    fb_data_t fb;
    fb.width = image_matrix->w;
    fb.height = image_matrix->h;
    fb.data = image_matrix->item;
    fb.bytes_per_pixel = 3;
    fb.format = FB_BGR888;
    fb_gfx_print(&fb, (fb.width - (strlen(str) * 14)) / 2, 10, color, str);
}
//===============================================================================================================//
int rgb_printf(dl_matrix3du_t *image_matrix, uint32_t color, const char *format, ...)
{
    char loc_buf[64];
    char * temp = loc_buf;
    int len;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    len = vsnprintf(loc_buf, sizeof(loc_buf), format, arg);
    va_end(copy);
    if(len >= sizeof(loc_buf)){
        temp = (char*)malloc(len+1);
        if(temp == NULL) {
            return 0;
        }
    }
    vsnprintf(temp, len+1, format, arg);
    va_end(arg);
    rgb_print(image_matrix, color, temp);
    if(len > 64){
        free(temp);
    }
    return len;
}
//===============================================================================================================//
void draw_face_box(dl_matrix3du_t *image_matrix, box_array_t *boxes, int face_id)
{
    int x, y, w, h, i;
    uint32_t color = FACE_COLOR_YELLOW;
    if(face_id < 0){  
      color = FACE_COLOR_RED;

      //tft.setCursor(5, 245, 4);      // Set "cursor" at top left corner of display (0,0) and select font 4
      //tft.setTextColor(TFT_RED, TFT_BLACK); // Set the font colour to be white with a black background
      //tft.printf("... Face Not Match...");  // We can now plot text on screen using the "print" class  
    //tft.printf("... Matched ID:%u ...", matched_id);
    } 
    //else if(face_id > 0){  
    //  color = FACE_COLOR_GREEN;   
      
    //}
  

        
}
//===============================================================================================================//
//===============================================================================================================//
void readSFace(fs::FS &fs, const char * path, int lineNum)
{
    char ch = 0;
    int i = 0;
    File file = fs.open(path);
    
    file.seek(lineNum);
    while(file.available()){
      ch = file.read(); 
      fac_buf[i++] = ch;
      if(ch == '#')//((1024*6)+1)
      { fac_buf[i++] = 0; break;}
    }

}
//===============================================================================================================//
int8_t recog_fc(dl_matrix3du_t *algined_face)
{
    int val = false;
    fptp_t similarity = 0;
    fptp_t max_similarity = -1;
    int8_t matched_id = -1;
    dl_matrix3d_t *face_id = NULL;
    
    float f = 0, ff = 0;
    int k = 0,l = 0,len = 0;
    int i= 0,j=0, m =0;

    fptp_t l2_norm_1 = 0;
    fptp_t l2_norm_2 = 0;
    fptp_t dist = 0;

    face_id = get_face_id(algined_face);
    uint32_t drawTime = millis();
    do{
                
        dist = 0;
        //------------------------------------------------------------------
        //for ( k = 0; k < USER_ID_VECTOR_SIZE; k++)
        m=0;f=0;ff=0;j=0;
        for ( k = 0; k < VECTOR_ARRAY_SIZE; k++)
        {   
            ff += face_id->item[k]; 
            ++m; 
            if(m == VECTOR_BLOCK_SIZE){ 
                //euclidean_distance......
                f = face_ram_buf[i][j++];

                //printf("|| k:%d | f:%f | ff:%f  \n",k,f,ff); 
                
                l2_norm_1 += (f*f ); //=================
                l2_norm_2 += (ff*ff);
                
                m=0;f=0;ff=0;
            }
        }
        l2_norm_1 = sqrt(l2_norm_1);l2_norm_2 = sqrt(l2_norm_2); 
        //for ( k = 0; k < USER_ID_VECTOR_SIZE; k++)
        m=0;f=0;ff=0;j=0;
        for ( k = 0; k < VECTOR_ARRAY_SIZE; k++)
        {   
            ff += face_id->item[k]; 
            ++m; 
            if(m == VECTOR_BLOCK_SIZE){
                //euclidean_distance......
                f = face_ram_buf[i][j++];
                fptp_t tmp = (f/l2_norm_1)-(ff/l2_norm_2);
                dist += (tmp*tmp);

                m=0;f=0;ff=0;
            }
        }
        similarity = dist;
        if( (dist > max_similarity) && (dist < FACE_REC_THRESHOLD)){ val=true; break;  }
        //------------------------------------------------------------------
    }while(i++ < face_ram_buf_count);
    dl_matrix3d_free(face_id);    if(val==false) i = i-1;   
    
    //printf("Userid:%d | dist:%06.2f \n",i, dist);     
    //printf("Executon Time: %d \n",( millis() - drawTime));
    
    if(val==true) return i; //return (i<1)?0:(i-1);
    else  return -1;
}
//===============================================================================================================//
int run_face_recog(dl_matrix3du_t *image_matrix, box_array_t *net_boxes){
   
    dl_matrix3du_t *aligned_face = NULL;
    int matched_id = 0;
    int i = 0;
    aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
    if(!aligned_face){
        //Serial.println("Could not allocate face recognition buffer");
        return matched_id;
    }
    if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK)
    {
        //----------------------------------------------------------------------------------------
        if(enroll_enable && (face_ram_buf_count < USER_ENROLL_COUNT)){  //enroll_enable  is_enrolling  

            dl_matrix3d_t *new_id = get_face_id(aligned_face);
            Serial.printf( "\nFACE_BUFFER_LENGTH:%d\n",new_id->c  ); 
            memset(fac_buf,0,sizeof(fac_buf));

            int m=0;float f=0; int j=0;
            for(int i=0; i<= VECTOR_ARRAY_SIZE; i++ ){  //VECTOR_ARRAY_SIZE 512    VECTOR_BLOCK_SIZE
                 f +=*(new_id->item + i);
                 ++m; 
                 if(m == VECTOR_BLOCK_SIZE){ 
                    face_ram_buf[face_ram_buf_count][j++] = f;
                    sprintf(s,"%06.2f",f);
                    strcat(fac_buf, s); 
                    m=0;f=0; 
                 }
            }
                     
            enroll_enable = 0;  sample_face_count = 0; face_ram_buf_count++;
            Serial.printf("face_enroll_data_appendF.........");
            strcat(fac_buf, "##"); 
            Serial.println(fac_buf);  
            Serial.printf("\n\nFace_ram_buf_count:%d \n",face_ram_buf_count);  
         
        }
         //----------------------------------------------------------------------------------------
        if(scan_enable){
                 //-----------------------------------------------------------------------------------------------
                matched_id = recog_fc(aligned_face);
                //-----------------------------------------------------------------------------------------------
                if (matched_id >= 0) {
                    tft.setCursor(5, 245, 4);      // Set "cursor" at top left corner of display (0,0) and select font 4
                    tft.setTextColor(TFT_GREEN, TFT_BLACK); // Set the font colour to be white with a black background
                    tft.printf("... Matched ID:%u ...", matched_id); // We can now plot text on screen using the "print" class  
                    //tft.printf("  Face Not Aligned  ");                             
                    //rgb_printf(image_matrix, FACE_COLOR_GREEN, "Matched_id %u", matched_id);
                    //Serial.printf("Matched Face ID: %u\n", matched_id);
                    //matched_id = 1;
                } else {
                    //Serial.println("No Match Found");
                    matched_id = -1; 
                }
        }
       //----------------------------------------------------------------------------------------
    }else {     

      //tft.setCursor(5, 245, 4);      // Set "cursor" at top left corner of display (0,0) and select font 4
      //tft.setTextColor(TFT_YELLOW, TFT_BLACK); // Set the font colour to be white with a black background
      //tft.printf("... Face Not Align...");  // We can now plot text on screen using the "print" class  
    //tft.printf("... Matched ID:%u ...", matched_id);                                           
      //rgb_print(image_matrix, FACE_COLOR_YELLOW, "Face Not Aligned");
      //Serial.println("Face Not Aligned");   
    }
    dl_matrix3du_free(aligned_face);
    return matched_id;
}
//===============================================================================================================//
//===============================================================================================================
esp_err_t face_detection (void)
{
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    dl_matrix3du_t *image_matrix = NULL;
    bool detected = false;
    int face_id = 0;
    uint16_t w = 0, h = 0;
    //-------------------------------------------------------------------------------------------------------
    while(true)
    {
          detected = false;
          face_id = 0;
          fb = esp_camera_fb_get();
          if (!fb) {  res = ESP_FAIL;     } 
          else{
              image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
              if (!image_matrix) { res = ESP_FAIL;   } 
              else{
                  if(!fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item)){   res = ESP_FAIL; } 
                  else {
                      box_array_t *net_boxes = NULL;
                      if(detection_enabled){
                          //---------------------------------------------------------------------------------
                          net_boxes = face_detect(image_matrix, &mtmn_config);
                          //---------------------------------------------------------------------------------
                          if(net_boxes){  detected = true;
                              //---------------------------------------------------------------------------------
                              face_id = run_face_recog(image_matrix, net_boxes);
                              //---------------------------------------------------------------------------------
                              draw_face_box(image_matrix, net_boxes, face_id);
                              //free(net_boxes->score);
                              //free(net_boxes->box);
                              //free(net_boxes->landmark);
                              free(net_boxes);
                          }
                      }
                  }
                  DisplayMatrixImage(fb->width,fb->height,image_matrix->item);
                  dl_matrix3du_free(image_matrix);
              }
        }
        //-------------------------------------------------------------------------------------------------------
        if(fb){   esp_camera_fb_return(fb);   fb = NULL;     } 
        if(res != ESP_OK){    break;     }
        //-------------------------------------------------------------------------------------------------------
        //-------------------------------------------------------------------------------------------------------
        if(face_id){
            //Serial.printf("Face: %s%d\n",(detected)?"DETECTED ":"", face_id    );
        }
        if ( Serial.available() ) {   
            chh = Serial.read();  //Serial.write( ch );  
            //if('v' == chh){  Serial.println("video Enable.....\n"); }
            

            //--------------------------------------------------------------------------------
            if('u' == chh){  Serial.println("Upload Face ....\n");
            
                  face_ram_buf_count=0;
                  int i=0;float f=0; int j=0;
                  for(i = 0; i < USER_ENROLL_COUNT; i++ ){
                      //Serial.println("\n.....................\n");
                      memset(fac_buf,0, sizeof(fac_buf));
                      readSFace(STORAGE_DEVICE, "/face_id.txt",i*((USER_ID_VECTOR_SIZE*6)+2));
                      int ln = strlen(fac_buf);
                      
                      //printf("\nstrlen(fac_buf):%d \n",ln);
                      if(ln < 1)break;

                      face_ram_buf_count++;
                      for(j = 0; j < USER_ID_VECTOR_SIZE; j++ ){
                          strncpy(s, &fac_buf[(j*6)], 6);
                          float f=atof(s);
                          //printf("%06.2f",f); 
                          face_ram_buf[i][j] = f;
                      }
                   }
            }
            //--------------------------------------------------------------------------------
            if('w' == chh){  Serial.println("\nDownLoad to vfss ....\n");
                if(face_ram_buf_count < USER_ENROLL_COUNT){
                      deleteF(STORAGE_DEVICE, "/face_id.txt");
                      int i=0;float f=0; int j=0;
                      for(i = 0; i < face_ram_buf_count; i++ ){
                          memset(fac_buf,0, sizeof(fac_buf));
                          for(j = 0; j < USER_ID_VECTOR_SIZE; j++ ){
                              f = face_ram_buf[i][j]; 
                              sprintf(s,"%06.2f",f);
                              strcat(fac_buf, s); 
                          }
                          strcat(fac_buf, "##"); 
                          //Serial.println(fac_buf);
                          appendF(STORAGE_DEVICE, "/face_id.txt",fac_buf);  
                      }
                 }
            } //SPIFFS
        }
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        if(conn_flag == 0){
            //------------------------------------------------------------------------//
            //wify_association();
            //------------------------------------------------------------------------//
            if (!client.connect(RemoteIP, atoi(RemotePort))) {
                delay(1000);
            }
            else{
                Serial.println("Remote Connected...");  
                conn_flag = 1;
            }
        }  
        if (!client.connected()) {
            Serial.println("Remote not Connected...");
            client.stop();
            conn_flag = 0;
        }
        else{
            if ( client.available() ) { ch = static_cast<char>(client.read());  Serial.write( ch ); 
                if('s' == ch){  Serial.println("Scan Enable......\n");scan_enable = 1; 
                    tft.setCursor(5, 280, 4);      // Set "cursor" at top left corner of display (0,0) and select font 4
                    tft.setTextColor(TFT_YELLOW, TFT_BLACK); // Set the font colour to be white with a black background
                    tft.printf("... Scan  Enable  ...");  // We can now plot text on screen using the "print" class  
                }
                if('e' == ch){  Serial.println("Enroll Enable....\n");enroll_enable = 1;scan_enable = 0; 
                    tft.setCursor(5, 280, 4);      // Set "cursor" at top left corner of display (0,0) and select font 4
                    tft.setTextColor(TFT_YELLOW, TFT_BLACK); // Set the font colour to be white with a black background
                    tft.printf("... Enroll Enable ...");  // We can now plot text on screen using the "print" class  
                }
                if('d' == chh){  Serial.println("deleteF Flash ....\n");deleteF(STORAGE_DEVICE, "/face_id.txt"); } //SPIFFS
                if('r' == ch){  Serial.println(" : readF Flash ....\n");readF(STORAGE_DEVICE, "/face_id.txt");  }  //SPIFFS
            }
        }
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    }
    return res;
}
//===============================================================================================================

//===============================================================================================================//
//===============================================================================================================//