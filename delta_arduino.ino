#include <Servo.h>

//Z değer aralığı -20,-35 arasında.
//XY değer aralığı -18,18 arasında.

Servo servo1,servo2,servo3;
// Delta Robotun geometrisi
#define e  35.0   
#define f  54.0   
#define re  30.2  
#define rf  15.6  
// trigonometrik hesaplamaların karşılıkları
#define sqrt3  1.732050808
#define pi  3.141592653
#define sin120  0.866025404
#define cos120  -0.5
#define tan60  1.732050808
#define sin30  0.5
#define tan30  0.577350269

int dly=100; // Gecikme değeri ne kadar artarsa o kadar yavaş çizer.
int asrv1,asrv2,asrv3; //Motorların eski açı değerleri.
int d_ang1,d_ang2,d_ang3;//Motorların eski açı değerleri ile yeni açı değerleri arasındaki fark. Deltası.

void setup() {
        Serial.begin(9600);//Seri port 9600 de açık.
        servo1.attach(9);// Kırmızı motor 9
        servo2.attach(10);// Sarı motor 10
        servo3.attach(11);// Yeşil motor 11
        servo1.write(90);//1. motor 90
        servo2.write(90);//2. motor 90
        servo3.write(90);//3. motor 90 derece.

}

void loop() {
  char chr;
  chr=Serial.read();
  if(chr=="I"){
    I();
  }

}

//Açı formülasyonu.
int AngleYZ(float x0, float y0, float z0) {
        float y1 = -0.5 * 0.57735 * f;
        y0 -= 0.5 * 0.57735 * e;
        // z = a + b*y
        float a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);
        float b = (y1-y0)/z0;
        // discriminant
        float d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf);
        if (d < 0) return -1;
        float yj = (y1 - a*b - sqrt(d))/(b*b + 1);
        float zj = a + b*yj;
        return 140-(180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1) ? 180.0 : 0.0));
}

// inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
// returned status: 0=OK, -1=imkansız pozisyon.
//xyz değerlerine göre oluşan açıları motorlara gönderme.
void xyz(float x0, float y0, float z0) {
           asrv1 = servo1.read();//1. motorun eski açı değeri
           asrv2 = servo2.read();//2. motorun eski açı değeri
           asrv3 = servo3.read();//3. motorun eski açı değeri
           d_ang1 = AngleYZ(x0, y0, z0) - asrv1;//1. motorun eski açısı ile yeni açısı arasındaki fark
           d_ang2 = AngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0) - asrv2;//2. motorun eski açısı ile yeni açısı arasındaki fark
           d_ang3 = AngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0) - asrv3;//3. motorun eski açısı ile yeni açısı arasındaki fark
           //Eski değerleri yeni değerlere ulaştırmak için 10 parçaya bölüyoruz.
           //i değeri her arttığında iki açı arasındaki farkın 10 da 1 i kadar eski açıya ekleniyor.
           //bu 10 kere tekrarlanınca belirlenmiş süre içinde yeni açı değerine ulaşmış oluyor.
           for (int i = 0; i < 10; i++) {
                asrv1 += d_ang1/10;
                asrv2 += d_ang2/10;
                asrv3 += d_ang3/10;
                servo1.write(asrv1);
                servo2.write(asrv2);
                servo3.write(asrv3);
                delay(dly);
           }


}

void I(){
        //Düz çizgi.
        xyz(0,-18,-30);//x=0,y=-18,z=-30
        delay(1000);//1 saniye bekleme.
        xyz(0,18,-30);//x=0,y=18,z=-30
        delay(1000);//1 saniye bekleme.
}

}

