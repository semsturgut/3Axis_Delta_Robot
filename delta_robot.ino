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

int dly=80; // Gecikme değeri ne kadar artarsa o kadar yavaş çizer.
int asrv1,asrv2,asrv3; //Motorların eski açı değerleri.
int d_ang1,d_ang2,d_ang3; //Motorların eski açı değerleri ile yeni açı değerleri arasındaki fark. Deltası.
char inData[20]; // Toplam string uzunlugu
char inChar; // Gecici karakter
byte index = 0; // Index into array; where to store the character
int count=0;
int move_x = 10;
int dly_all=900;
int z_down=-35,z_up=-23;
int x=0,y=0;

void setup() {
        Serial.begin(9600); //Seri port 9600 de açık.
        servo1.attach(9); // Kırmızı motor 9
        servo2.attach(10); // Sarı motor 10
        servo3.attach(11); // Yeşil motor 11
        inData[0]="1";

}

void loop() {
        char chr;
        while(Serial.available() > 0) { // Serialden bir sey yaziliyorsa while a gir
                if(index < 19) { //19 dan buyuk olmasın
                        inChar = Serial.read(); // karakteri oku
                        inData[index] = inChar; // karakteri kaydet
                        index++; // indexi 1 arttır
                        inData[index] = '\0'; // son degere bitis ekle
                }
                count=0;
        }

        x=-18; y=-10;
        xyz(x,y,z_up);
        delay(dly_all);
        index=0;
        while(1) {
                if (count==1) { //yazi bir kere yazilmis ise bir daha yazma
                        break; //donguden yeni yazi gelene kadar cik
                }
                if (inData[index]=='\0') { //son degere gelindiginde
                        count++; //degeri bir arttir
                        break; //donguden cikis
                }


                if(inData[index]=='a') {
                        Serial.print("a");
                }
                if(inData[index]== 'b') {
                        Serial.print("b");
                }
                if(inData[index]== 'c') {
                        Serial.print("c");
                }
                if(inData[index]== 'd') {
                        Serial.print("d");
                }
                if(inData[index]== 'e') {
                        Serial.print("e");
                }
                if(inData[index]== 'f') {
                        Serial.print("f");
                }
                if(inData[index]== 'g') {
                        Serial.print("g");
                }
                if(inData[index]== 'h') {
                        Serial.print("h");
                }
                if(inData[index]== 'i') {
                        Serial.print("i");
                        xyz(x,y,z_up); //x,y,-20
                        delay(dly_all);
                        xyz(x,y,z_down); //x,y,-30
                        delay(dly_all);
                        xyz(x,y+20,z_down); //x,y+20,-30
                        delay(dly_all);
                        xyz(x,y+20,z_up); //x,y+20,-20
                        delay(dly_all);
                        x=x+move_x; //x e onceki degerine gore yeni deger ekleniyor

                }
                if(inData[index]== 'j') {
                        Serial.print("j");
                }
                if(inData[index]== 'k') {
                        Serial.print("k");
                }
                if(inData[index]== 'l') {
                        Serial.print("l");
                        xyz(x,y+20,z_up);
                        delay(dly_all);
                        xyz(x,y+20,z_down);
                        delay(dly_all);
                        xyz(x,y,z_down);
                        delay(dly_all);
                        x=x+move_x+5;
                        xyz(x,y,z_down);
                        delay(dly_all);
                        xyz(x,y,z_up);
                        delay(dly_all);
                        x=x+move_x;

                }
                if(inData[index]== 'm') {
                        Serial.print("m");
                }
                if(inData[index]== 'n') {
                        Serial.print("n");
                }
                if(inData[index]== 'o') {
                        Serial.print("o");
                }
                if(inData[index]== 'p') {
                        Serial.print("p");
                }
                if(inData[index]== 'r') {
                        Serial.print("r");
                }
                if(inData[index]== 's') {
                        Serial.print("s");
                }
                if(inData[index]== 't') {
                        Serial.print("t");
                }
                if(inData[index]== 'u') {
                        Serial.print("u");
                }
                if(inData[index]== 'v') {
                        Serial.print("v");
                }
                if(inData[index]== 'y') {
                        Serial.print("y");
                }
                if(inData[index]== 'z') {
                        Serial.print("z");
                }
                if(inData[index]== ' ') {
                        Serial.print(" ");
                }
                index++;
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
        asrv1 = servo1.read();   //1. motorun eski açı değeri
        asrv2 = servo2.read();   //2. motorun eski açı değeri
        asrv3 = servo3.read();   //3. motorun eski açı değeri
        d_ang1 = AngleYZ(x0, y0, z0) - asrv1;   //1. motorun eski açısı ile yeni açısı arasındaki fark
        d_ang2 = AngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0) - asrv2;   //2. motorun eski açısı ile yeni açısı arasındaki fark
        d_ang3 = AngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0) - asrv3;   //3. motorun eski açısı ile yeni açısı arasındaki fark
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
