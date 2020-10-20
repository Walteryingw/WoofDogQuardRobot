void control() {    //蓝牙串口命令,也适用WIFI无线串口

  /*if (mode == 1) {
    int cm = Distance();
    if (cm > 30) { //大于30CM直走
      walk();  //直走
    } else if (cm >= 0 && cm <= 30) {
      stand();
               //BACK, TOLEFT/TORIGHT? HEIGHT?
    }
    }*/
  if (SerialBT.available() > 0) {
    int val = SerialBT.read();
    if (val >= 0 && val <= 100) {
      speed = float(val * 0.001);
    } else if (val >= 150 && val <= 250) {
      S0 = int(val - 200);
    }
    else if (val == 103) {
      mode = 1;
    } else {
      switch (val) {
        case 105:     //PWM停
          k = "stop";
          break;
        case 104:     //PWM唤醒
          k = "start";
          break;
        case 101:    //小跑
          k = "trot";
          break;
        case 102: //爬行步态
          k = "walk";
          break;

        case 127://
          k = "update"; // 在线升级
          break;
        case 110://自稳模式
          k = "mpu";
          break;
        case 106:   //站立
          k = "stand";
          break;
        case 111://静姿态模式站立
          k = "balance";
          break;
        case 107:  //校准
          k = "calib";
          break;
        case 116://
          k = "rests"; // 休息
          break;
        case 115:   //  右移
          k = "toRT";
          break;
        case 114:   // 左移
          k = "toLT";
          break;
        case 108:    //高
          k = "up";
          break;
        case 109:   //变矮
          k = "down";
          break;
        case 112:   //左转
          k = "toLt";
          break;
        case 113:  //右转
          k = "toRt";
          break;
        case 128:   //左摆
          k = "zd";
          break;
        case 129:   //右摆
          k = "yd";
          break;
        case 121:   //前摆
          k = "qd";
          break;
        case 122:   //后摆
          k = "hd";
          break;
        case 123:   //左前低
          k = "zqd";
          break;
        case 124:   //右前低
          k = "yqd";
          break;
        case 125:   //左后低
          k = "zhd";
          break;
        case 126:   //右后低
          k = "yhd";
          break;
        default:
          break;
      }

      mode = 0;
    }

  }

  if (k == "update") {
    //wificon();  //开启WIFI
    OTA();  //OTA升级功能
    ArduinoOTA.handle();
  }
  else if (k == "stop") { //停止
    if (ps == 1) {
      pwm.sleep();
      ps = 0;
    }
  }
  else if (k == "start") { //开始
    pwm.wakeup();
    ps = 1;
  }

  else if (k == "trot") { //小跑

    z = 0;
    zz = 0;
    t = t + speed;
    trot();

  }
  else if (k == "walk") { //爬行

    z = 0;
    zz = 0;
    t = t + speed;
    walk();
  }
  else if (k == "toRT") { //右平移
    S0 == 0;
    zz = 12;
    t = t + speed;
    turn2();
  }
  else if (k == "toLT") { //左平移
    S0 == 0;
    zz = -12;
    t = t + speed;
    turn2();
  }

  else if (k == "mpu") { //自稳
    // mpu();
    balance2();//平衡方程
  }
  else if (k == "toLt") { //左转
    //  S0=30;
    z = -12;     //通过 z 的改变来改变转弯角度
    zz = 0;
    t = t + speed;
    trot();
  }
  else if (k == "toRt") { //右转
    // S0=30;
    z = 12;
    zz = 0;
    t = t + speed;
    trot();
  }
  else if (k == "balance") { //调用DMP数据
    balance();  //静姿态
  }
  else if (k == "up") { //加高
    Ht = Ht + 2;
    delay(50);
    baidong2();
    if (Ht > 130)
      Ht = 130;
  }
  else if (k == "down") { //变矮
    Ht = Ht - 2;
    delay(50);
    baidong2();
    if (Ht < 50)
      Ht = 50;
  }
  else if (k == "zd") { //左摆
    DX = DX + 1;
    delay(20);
    zitai();

  }
  else if (k == "yd") { //右摆
    DX = DX - 1;
    delay(20);
    zitai();

  }
  else if (k == "qd") { //前摆
    AX = AX + 1;
    delay(20);
    zitai();
  }
  else if (k == "hd") { //后摆
    AX = AX - 1;
    delay(20);
    zitai();
  }
  else if (k == "zqd") { //左前低
    z = z - 1;
    delay(20);
    zitai();
  }
  else if (k == "yqd") { //右前低
    z = z + 1;
    delay(20);
    zitai();
  }
  else if (k == "zhd") { //左后低
    z = z - 1;
    AX = AX - 1;
    delay(20);
    zitai();
  }
  else if (k == "yhd") { //左前低
    z = z + 1;
    AX = AX + 1;
    delay(20);
    zitai();
  }
  else if (k == "calib") {
    calib();
  }
  else if (k == "stand") {
    // S0=0;
    z = 0;
    zz = 0;
    AX = 0;
    DX = 0;
    //  Hc=96;
    stand();
  }

  else if (k == "jump") {
    jump();
  }
  else if (k == "rests") {
    rests();
  }
  else if (k == "scalib") {
    j[0] = 8;
    j[1] = 8;
    //  savedata();
  }

  Serial.print(speed, 3);
  Serial.print("   ");
  Serial.println(k);
}
