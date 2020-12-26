#include <jni.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include "android/log.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>


static const char *TAG="serial_port";
#define LOGI(fmt, args...) __android_log_print(ANDROID_LOG_INFO,  TAG, fmt, ##args)
#define LOGD(fmt, args...) __android_log_print(ANDROID_LOG_DEBUG, TAG, fmt, ##args)
#define LOGE(fmt, args...) __android_log_print(ANDROID_LOG_ERROR, TAG, fmt, ##args)
static speed_t getBaudrate(jint baudrate)
{
    switch(baudrate) {
        case 1200: return B1200;
        case 1800: return B1800;
        case 2400: return B2400;
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 500000: return B500000;
        case 576000: return B576000;
        case 921600: return B921600;
        case 1000000: return B1000000;
        case 1152000: return B1152000;
        default: return -1;
    }
}

/*
 * Java_com_smartlab_blue_serialdemo_SerialPort_open中的com_smartlab_blue_serialdemo是包名
 * SerialPort_open是SerialPort类中的private native static FileDescriptor open(...)方法，需要用"_"连接不能用"."
 * 有的可能直接在SerialPort类中进行Alt+Enter就可以在这个文件里面创建一个open方法，只需把这个类中我写的方法名换一下就行了，
 * 有的就需要按照我上面说的方法改方法名字。   下面的close方法配置一样
 * */
JNIEXPORT jobject JNICALL
Java_com_serialportproj_serial_SerialPort_open(JNIEnv *env, jclass type, jstring path, jint baudrate, jint flags) {
    int fd;
    speed_t speed;      //引用#include <termios.h>
    jobject mFileDescriptor;

    /* Check arguments */
    {
        speed=getBaudrate(baudrate);
        if (speed==-1){

            LOGE("Invalid baudrate");
            return NULL;
        }
    }

    /* 开启 设备 */
    {
        jboolean iscapy;
        const char *path_utf = (*env)->GetStringUTFChars(env, path, &iscapy);
        LOGD("Opening serial port %s with flags 0x%x", path_utf, O_RDWR | flags);
        fd=open(path_utf,O_RDWR | flags | O_NDELAY);//O_RDWR 引入     #include <fcntl.h>
        LOGD("open() fd = %d", fd);
        (*env)->ReleaseStringUTFChars(env, path, path_utf );
        if (fd==-1){
            /* 引发异常 */
            LOGE("Cannot open port");
            /* TODO: throw an exception */
            return NULL;
        }

    }

    /* 配置 设备 */
    {
        struct termios cfg;
        LOGD("Configuring serial port");
        if (tcgetattr(fd,&cfg))
        {
            LOGE("tcgetattr() failed");
            close(fd);
            /* TODO: throw an exception */
            return NULL;
        }

        cfmakeraw(&cfg);
        cfsetispeed(&cfg, speed);
        cfsetospeed(&cfg, speed);

        if (tcsetattr(fd, TCSANOW, &cfg))
        {
            LOGE("tcsetattr() failed");
            close(fd);
            /* TODO: throw an exception */
//            return NULL;
        }
    }

    /* 创建一个相应的文件描述符 */
    {
        jclass  cFileDescriptor=(*env)->FindClass(env,"java/io/FileDescriptor");
        jmethodID iFileDescriptor=(*env)->GetMethodID(env,cFileDescriptor,"<init>","()V");
        jfieldID descriptorID = (*env)->GetFieldID(env, cFileDescriptor, "descriptor", "I");
        mFileDescriptor = (*env)->NewObject(env, cFileDescriptor, iFileDescriptor);
        (*env)->SetIntField(env, mFileDescriptor, descriptorID, (jint)fd);

    }

    return mFileDescriptor;
}

JNIEXPORT void JNICALL
Java_com_serialportproj_serial_SerialPort_close(JNIEnv *env, jobject instance) {
    jclass SerialPortClass = (*env)->GetObjectClass(env, instance);
    jclass FileDescriptorClass = (*env)->FindClass(env, "java/io/FileDescriptor");

    jfieldID mFdID = (*env)->GetFieldID(env, SerialPortClass, "mFd", "Ljava/io/FileDescriptor;");
    jfieldID descriptorID = (*env)->GetFieldID(env, FileDescriptorClass, "descriptor", "I");

    jobject mFd = (*env)->GetObjectField(env, instance, mFdID);
    jint descriptor = (*env)->GetIntField(env, mFd, descriptorID);

    LOGD("close(fd = %d)", descriptor);
    close(descriptor);
}