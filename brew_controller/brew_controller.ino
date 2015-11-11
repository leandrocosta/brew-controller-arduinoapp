#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>

const int BUFFSIZE = 200;
char req[BUFFSIZE];
char resp[BUFFSIZE];

class HeatController {
  public:
    unsigned int pinSSR = 0, pinDS18B20 = 0;
    OneWire* oneWire = 0;
    DallasTemperature* sensors = 0;
    PID *pid = 0;
    double Input = 0, Output = 0, Setpoint = 0;
    unsigned int sampleTime = 0, windowSize = 0;
    unsigned long windowStartTime = millis();
    unsigned long lastReport = windowStartTime;
    boolean running = false;
    byte outputSSR = LOW;

  public:
    boolean setParameters(unsigned int pinSSR, unsigned int pinDS18B20, double Kp, double Ki, double Kd, double Setpoint, double Output, unsigned int SampleTime, unsigned int windowSize) {
      this->pinSSR = pinSSR;
      if (this->pinSSR != 0) {
        pinMode(this->pinSSR, OUTPUT);
      }

      this->pinDS18B20 = pinDS18B20;
      if (this->oneWire) {
        delete this->oneWire;
        this->oneWire = 0;
      }
      if (this->sensors) {
        delete this->sensors;
        this->sensors = 0;
      }

      if (this->pinDS18B20 != 0) {
        this->oneWire = new OneWire(this->pinDS18B20);
        this->sensors = new DallasTemperature(this->oneWire);
        this->sensors->begin();
      }

      if (this->pid) {
        delete this->pid;
        this->pid = 0;
      }

      this->Setpoint = Setpoint;
      this->Output = Output;
      this->sampleTime = SampleTime;
      this->windowSize = windowSize;
      this->pid = new PID(&(this->Input), &(this->Output), &(this->Setpoint), Kp, Ki, Kd, DIRECT);
      this->pid->SetSampleTime(this->sampleTime);
      this->pid->SetOutputLimits(0, this->windowSize);
      this->pid->SetMode(AUTOMATIC);

      return true;
    }

    boolean setPinDS18B20(unsigned int pinDS18B20) {
      this->pinDS18B20 = pinDS18B20;
      if (this->oneWire) {
        delete this->oneWire;
      }
      this->oneWire = new OneWire(pinDS18B20);

      if (this->sensors) {
        delete this->sensors;
      }
      this->sensors = new DallasTemperature(this->oneWire);
      this->sensors->begin();

      return true;
    }

    void calcTemp() {
      if (this->sensors) {
        this->sensors->requestTemperatures();
        this->Input = this->sensors->getTempCByIndex(0);
      }
    }

    void runPID() {
      this->pid->Compute();
      unsigned long now = millis();

      if (now - this->windowStartTime > this->windowSize) {
        this->windowStartTime += this->windowSize * (now - this->windowStartTime) / this->windowSize;
      }

      if (this->Output > now - this->windowStartTime) {
        this->outputSSR = HIGH;
        digitalWrite(this->pinSSR, HIGH);
      }
      else {
        this->outputSSR = LOW;
        digitalWrite(this->pinSSR, LOW);
      }
    }

    void play() {
      this->running = true;
    }

    void stop() {
      this->outputSSR = LOW;
      digitalWrite(this->pinSSR, LOW);
      this->running = false;
    }

    void reportStatus(unsigned int idx, unsigned long period) {
      unsigned long now = millis();
      if (now < this->lastReport + period) {
        return;
      }

      Serial.print("{\"idx\":");
      Serial.print(idx);
      Serial.print(",\"pinSSR\":");
      Serial.print(this->pinSSR);
      Serial.print(",\"pinDS18B20\":");
      Serial.print(this->pinDS18B20);
      Serial.print(",\"kp\":");
      Serial.print(this->pid->GetKp());
      Serial.print(",\"ki\":");
      Serial.print(this->pid->GetKi());
      Serial.print(",\"kd\":");
      Serial.print(this->pid->GetKd());
      Serial.print(",\"sampleTime\":");
      Serial.print(this->sampleTime);
      Serial.print(",\"windowSize\":");
      Serial.print(this->windowSize);
      Serial.print(",\"setpoint\":");
      Serial.print(this->Setpoint);
      Serial.print(",\"input\":");
      Serial.print(this->Input);
      Serial.print(",\"output\":");
      Serial.print(this->Output);
      Serial.print(",\"running\":");
      Serial.print(this->running);
      Serial.print(",\"outputSSR\":");
      Serial.print(this->outputSSR);
      Serial.println("}");
      Serial.flush();

      this->lastReport = now;
    }
};

HeatController heatCtrls[3];

void setup() {
  Serial.begin(9600);
  sendToSerial("{\"status\":\"running\"}");
}

void loop() {
  //unsigned long start = millis();

  if (Serial.available() > 0) {
    handleRequest();
  }

  for (byte i = 0; i < 3; i++) {
    if (heatCtrls[i].sensors) {
      heatCtrls[i].calcTemp();
      if (heatCtrls[i].running) {
        heatCtrls[i].runPID();
      }
    }

    heatCtrls[i].reportStatus(i, 1000);
  }

  //unsigned long end = millis();
  //if (end < start + 1000) {
  //  delay(1000 - (end - start));
  //}
}

void handleRequest() {
  String str = Serial.readStringUntil('\n');
  str.toCharArray(req, BUFFSIZE);

  //sprintf(resp, "LOG: RECEIVED %s", req);
  //sendToSerial(resp);
  //memset(resp, 0, BUFFSIZE);

  switch (req[0]) {
    case 'S': // S 0 11 10 10000 0 0 10000 35.0
      handleCmdSet(req);
      break;
    case 'D': // D 0 10
      handleCmdPinDS18B20(req);
      break;
    case 'P': // P 0
      handleCmdPlay(req);
      break;
    case 'T': // T 0
      handleCmdStop(req);
      break;
    case 'E': // E 0
      handleCmdTemp(req);
      break;
    default:
      sprintf(resp, "{\"success\":false,\"msg\":\"unknown command\"}");
      sendToSerial(resp);
      break;
  }
}

void handleCmdSet(const char* req) {
  unsigned int idx, pinSSR, pinDS18B20, sampleTime, windowSize;
  char Kp[9], Ki[9], Kd[9], Setpoint[7], Output[9];

  sscanf(req, "S %u %u %u %s %s %s %u %u %s %s", &idx, &pinSSR, &pinDS18B20, &Kp, &Ki, &Kd, &sampleTime, &windowSize, &Setpoint, &Output);

  if (heatCtrls[idx].setParameters(pinSSR, pinDS18B20, atof(Kp), atof(Ki), atof(Kd), atof(Setpoint), atof(Output), sampleTime, windowSize)) {
    sprintf(resp, "{\"cmd\":\"set\",\"idx\":%u,\"success\":true}", idx);
    sendToSerial(resp);
  } else {
    sprintf(resp, "{\"cmd\":\"set\",\"idx\":%u,\"success\":false}", idx);
    sendToSerial(resp);
  }
}

void handleCmdPinDS18B20(const char* req) {
  unsigned int idx, pinDS18B20;
  sscanf(req, "D %u %u", &idx, &pinDS18B20);
  if (heatCtrls[idx].setPinDS18B20(pinDS18B20)) {
    sprintf(resp, "{\"cmd\":\"ds18b20\",\"idx\":%u,\"success\":true}", idx);
    sendToSerial(resp);
  } else {
    sprintf(resp, "{\"cmd\":\"ds18b20\",\"idx\":%u,\"success\":false}", idx);
    sendToSerial(resp);
  }
}

void handleCmdPlay(const char* req) {
  unsigned int idx;
  sscanf(req, "P %u", &idx);
  heatCtrls[idx].play();
  sprintf(resp, "{\"cmd\":\"play\",\"idx\":%u,\"success\":true}", idx);
  sendToSerial(resp);
}

void handleCmdStop(const char* req) {
  unsigned int idx;
  sscanf(req, "T %u", &idx);
  heatCtrls[idx].stop();
  sprintf(resp, "{\"cmd\":\"stop\",\"idx\":%u,\"success\":true}", idx);
  sendToSerial(resp);
}

void handleCmdTemp(const char* req) {
  unsigned int idx;
  sscanf(req, "E %u", &idx);
  Serial.print("{\"cmd\":\"temp\",\"idx\":");
  Serial.print(idx);
  Serial.print(",\"success\":true,\"value\":");
  Serial.print(heatCtrls[idx].Input);
  Serial.println("}");
}

void sendToSerial(const char* buff) {
  Serial.println(buff);
  Serial.flush();
}

