# ส่วนควบคุมหุ่น (robot_ctrl + teleop + robot_pose)

| ไฟล์ | หน้าที่ |
| --- | --- |
| `firmware/robot_ctrl/` | ESP32: ล้อ, servo 2 ตัว, ระบบความปลอดภัยทั้งหมด (protocol v3, UDP 4211) |
| `teleop.py` | ขับด้วยคีย์บอร์ดจาก PC + บันทึกวิดีโอ/log ของทุก run ลง `runs/` |
| `robot_pose.py` | หาตำแหน่ง/ทิศของหุ่นจาก AprilTag บนหลังหุ่น เป็น mm บนสนาม |
| `tests/test_robot_pose.py` | เทสต์สังเคราะห์ของ robot_pose |

`firmware/esp_link/` (ตัวรับเป้าหมายจาก vision) ยังอยู่เหมือนเดิม ใช้พอร์ต 4210 ไม่ชนกัน

## 1. Firmware

1. Arduino IDE: ติดตั้ง board **esp32 by Espressif เวอร์ชัน 3.x** และ library **ArduinoJson 7**
2. `firmware/robot_ctrl/secrets.example.h` → คัดลอกเป็น `secrets.h` แล้วใส่ Wi-Fi
3. **แก้ `config.h` ให้ตรงกับสายที่ต่อจริงก่อน** (pin ทุกตัวในไฟล์เป็นค่าตัวอย่าง)
   - เลือกชนิด driver: `DRIVER_IN_IN_PWM` (TB6612/L298N) หรือ `DRIVER_TWO_PWM` (DRV8833/MX1508)
   - ปุ่มหยุดฉุกเฉินต่อระหว่าง `ESTOP_PIN` กับ GND
   - เริ่มที่ `MAX_DUTY 0.60` ก่อน
4. Upload แล้วเปิด Serial Monitor 115200 จด IP ที่ขึ้นมา

สถานะ: `IDLE → (g) RUNNING → 5 นาที → DONE`; ปุ่มหรือ `x` → `ESTOP` (ค้างจนกด `r` และปล่อยปุ่มแล้ว)

- ล้อหมุนได้เฉพาะ `RUNNING` และต้องได้คำสั่ง drive ทุก ≤300 ms ไม่งั้นหยุดเอง
- servo ขยับได้ใน `IDLE` (ไว้ calibrate) และ `RUNNING`; ใน `DONE/ESTOP` ค้างตำแหน่ง (ไม่ปล่อยหิน)
- ESP32 ส่งสถานะกลับไปที่ PC ทุก 200 ms (teleop แสดงบนจอ)
- **ตอนเปิดเครื่อง servo จะกระโดดไปที่ `SERVO_START_DEG`** ตั้งค่านี้ให้เป็นท่าที่ปลอดภัย

## 2. ลำดับทดสอบ (ยกล้อลอยก่อนเสมอ)

```bash
python teleop.py <ESP_IP> --no-log
```

1. ยังไม่กด `g` → กด `w` ต้อง**ไม่หมุน** (IDLE)
2. กด `g` → `w` ล้อทั้งสองต้องหมุนไปข้างหน้า ถ้าข้างไหนกลับ แก้ `L_INVERT/R_INVERT`
3. ปล่อยปุ่ม → หยุดภายใน ~0.6 s
4. กด `w` ค้าง แล้ว **ปิดโปรแกรม / ปิด Wi-Fi ของ PC** → ล้อต้องหยุดภายใน 0.3 s
5. กดปุ่ม e-stop ตอนล้อหมุน → หยุดทันที, จอขึ้น ESTOP; กด `r` ขณะยังกดปุ่มค้าง → ต้องไม่ reset
6. ทดสอบหมดเวลา: ตั้ง `RUN_TIME_MS` เป็น 20000 ชั่วคราว → ต้องเข้า DONE เอง (อย่าลืมคืนเป็น 300000)
7. servo: หาองศาด้วยคำสั่งดิบ แล้วเอาไปใส่ `GRIP_*`, `LIFT_*`, `SERVO_MIN/MAX_DEG`
   ```python
   from teleop import Link; l = Link('<ESP_IP>', 4211); l.send('servo', i=0, deg=90)
   ```
8. ถ้า ESP32 รีเซ็ตตอน servo ขยับ (Serial ขึ้น boot ใหม่) = ไฟตก → แยกไฟ servo, ต่อ GND ร่วม, ลด `SERVO_DEG_PER_SEC`

## 3. AprilTag

- พิมพ์ tag ตระกูล **36h11** (เช่น id 0) หลายขนาด 80/100/120 mm มีขอบขาวรอบ tag อย่างน้อย 1 ช่อง ติดให้**เรียบและขนานพื้น**
- **ขอบบนของ tag = ด้านหน้าหุ่น**
- ใส่ค่าที่วัดจริงใน `calib.json` → `robot_tag`:

```json
"robot_tag": {
  "id": 0, "family": "36h11",
  "size_mm": 100,               // ขนาดช่องดำด้านนอก ไม่รวมขอบขาว
  "height_mm": 150,             // ความสูงผิว tag จากพื้น
  "camera_height_mm": 2000,     // ความสูงเลนส์จากพื้น
  "camera_floor_xy_mm": null,   // จุดบนพื้นใต้กล้อง; null = กลางสนาม
  "grip_offset_mm": [120, 0],   // จากกลาง tag ไปปากแขน [ไปข้างหน้า, ไปทางขวา]
  "size_tolerance": 0.25,
  "footprint_mm": {"front": 170, "back": 110, "left": 105, "right": 105}
}
```
(ใน JSON จริงห้ามมีคอมเมนต์ `//`)

ทดสอบ: `python robot_pose.py 1` จะพิมพ์อัตราการเจอ tag และขนาด tag เป็นพิกเซล
เลือก tag ขนาดเล็กสุดที่เจอ ~100% ทั้ง 4 มุมสนามตอนหุ่นวิ่ง ถ้าขึ้น `size ... != ...`
แปลว่า `height_mm`/`camera_height_mm` หรือการ calibrate มุมสนามผิด

ตรวจความแม่น: วางหุ่นที่จุดที่วัดด้วยตลับเมตร (4 มุม + กลาง) แล้วเทียบกับ x,y ที่พิมพ์ออกมา

## 4. บันทึก run (สำหรับวิเคราะห์ และ LeRobot ภายหลัง)

```bash
python teleop.py <ESP_IP> --camera 1
```

ได้ `runs/<เวลา>/`:
- `video.mp4` เฟรมกล้องดิบ → เล่นซ้ำด้วย `detect_live.py --video` หรือ `robot_pose.py --video` ได้
- `log.jsonl` 20 แถว/วินาที: `t`, `frame`, `action {l, r}`, `events` (start/stop/grip/lift), `status` (state, servo, …), `pose`
- `meta.json` ค่าตั้งและ calib ที่ใช้ตอนนั้น

แต่ละแถวคือ (observation, action) ของ 1 timestep ตรงกับที่ LeRobotDataset ต้องการ ภายหลังเขียน converter ได้โดยไม่ต้องเก็บข้อมูลใหม่
