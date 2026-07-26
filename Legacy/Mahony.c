#include "Mahony.h"
#include <math.h>
#include <stdint.h>
//Mahony互补滤波算法，就是当imu静态时，只受到重力，用加速度修正角速度，动态时修正效果显然是不好的
  static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;   // 四元数，初始 = 无旋转
  static float exInt, eyInt, ezInt;               // 积分误差,e=error,Int=integral
  static float twoKp, twoKi;             // 控制参数,twoKp = 2 × Kp, twoKi = 2 × Ki


  void MahonyAHRSinit() //AHRS = Attitude and Heading Reference System就是姿态
  {
      twoKp   = 2.0f * 1.5f;             // Kp = 1.5（比例增益）
      twoKi   = 2.0f * 0.05f;            // Ki = 0.05（积分增益）
      q0 = 1; q1 = 0; q2 = 0; q3 = 0;   // 初始姿态 = 无旋转 (w=1, xyz=0)
      exInt = eyInt = ezInt = 0;         // 积分项清零
  }

/*
   brief：这是一个归一化函数，若采用欧拉积分法，每次会加上一个极小量，导致四元数偏离1，所以要归一化,但是这个会产生误差的
   但是还有指数映射的方法，这样就不用归一化了
*/
  static void normalize(void)
  {
      float n = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
      if (n < 1e-12f) {                  // 躺平了，救不回来了
          q0 = 1; q1 = 0; q2 = 0; q3 = 0;
          return;
      }
      float inv = 1.0f / n;
      q0 *= inv; q1 *= inv; q2 *= inv; q3 *= inv;
  }

  /* ═══════════════════════════════════════════════════════════════════════
   * MahonyAHRSupdate — 主更新
   *
   * 每帧调一次，做以下事：
   *   1. 归一化加速度
   *   2. 用当前四元数预测"重力在 body 系的方向"
   *   3. 计算预测方向 vs 实测方向的叉积 → 得到姿态误差 e
   *   4. 用 PI 控制器把 e 反馈到角速度上（校正陀螺漂移）
   *   5. 用校正后的角速度更新四元数 q_dot = 0.5 * q ⊗ ω
   *   6. 归一化四元数
   *
   * 参数:
   *   gx/gy/gz  — 陀螺 [rad/s]
   *   ax/ay/az  — 加速度 [m/s²]
   * ═══════════════════════════════════════════════════════════════════════ */
  void MahonyAHRSupdate(float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float dt_acc,float dt_gyro)
  {
      /* ── 1. 归一化加速度 ── */
      /* 把加速度变成单位向量，这样只保留方向信息，去掉幅值 */
      float n = sqrtf(ax*ax + ay*ay + az*az);
      if (n < 1e-12f) return;     // 加速度读数接近 0（传感器故障/自由落体），跳过
      float inv = 1.0f / n;
      ax *= inv; ay *= inv; az *= inv;

      /* ── 2. 预测重力在 body 系的方向 ── */
      //就是四元数旋转矩的逆乘[0,0,1]T（或者就用四元数乘法，左乘q并右乘q逆），旋转矩阵用四元数表示，变换完，g就在imu所在的参考系
      //v=vector
      float vx = 2.0f * (q1*q3 - q0*q2);        // = -R[2][0] (归一化后)
      float vy = 2.0f * (q0*q1 + q2*q3);        // = -R[2][1]
      float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3; // = -R[2][2] = 1-2(q1²+q2²)

      /* ── 3. 叉积误差 ── */
      /* e = a_meas × a_pred
         物理意义: 叉积结果的方向 = 需要绕哪个轴转才能纠正姿态
                   叉积结果的模长 = sin(误差角)，小角时 ≈ 角度误差 */
      float ex = ay*vz - az*vy;
      float ey = az*vx - ax*vz;
      float ez = ax*vy - ay*vx;

      /* ── 4. PI 控制 ── */
      /* I 项: 积分误差，缓慢积累用于估计陀螺零偏漂移 */
      exInt += ex * twoKi * dt_acc;      // = ex × Ki × dt
      eyInt += ey * twoKi * dt_acc;
      ezInt += ez * twoKi * dt_acc;

      /* 防积分饱和: 限制在 ±0.1 rad/s (≈ ±6 dps) */
      float lim = 0.1f;
      if (exInt >  lim) exInt =  lim;
      if (exInt < -lim) exInt = -lim;
      if (eyInt >  lim) eyInt =  lim;
      if (eyInt < -lim) eyInt = -lim;
      if (ezInt >  lim) ezInt =  lim;
      if (ezInt < -lim) ezInt = -lim;

      /* P 项(立即修正) + I 项(长期零偏) */
      /* ω_corrected = ω_gyro + Kp × e + ∫Ki×e dt
         注意这里是 +=, 所以 gx 传进来后直接被修改了（C 是按值传递的副本，没关系）*/
      gx += twoKp * ex + exInt;
      gy += twoKp * ey + eyInt;
      gz += twoKp * ez + ezInt;

      /* ── 5. 四元数更新 q_dot = 0.5 × q ⊗ ω ── */
      //使用欧拉积分，即talor的一阶项，丢弃高阶项
      /* q ⊗ ω 的展开式: */
      //TODO:使用更高精度的积分器
      float qa = q0, qb = q1, qc = q2;
      q0 += (-qb*gx - qc*gy - q3*gz) * 0.5f * dt_gyro;
      q1 += ( qa*gx + qc*gz - q3*gy) * 0.5f * dt_gyro;
      q2 += ( qa*gy - qb*gz + q3*gx) * 0.5f * dt_gyro;
      q3 += ( qa*gz + qb*gy - qc*gx) * 0.5f * dt_gyro;

      /* ── 6. 归一化 ── */
      normalize();
  }

    /*
    MahonyGetQuaternion — 取出四元数
    返回当前姿态的四元数 [w, x, y, z]
    滤波器内部状态的只读接口
    哎呦wc，这个排版这么这么史啊，不改了
    */
  void MahonyGetQuaternion(float *o0, float *o1, float *o2, float *o3)
  {
      *o0 = q0;
      *o1 = q1;
      *o2 = q2;
      *o3 = q3;
  }

  /* ═══════════════════════════════════════════════════════════════════════
   * MahonyGetEuler — 四元数 → 欧拉角 (ZYX 内旋)
   *
   *   Roll  (φ): 绕 X 轴转
   *   Pitch (θ): 绕 Y 轴转
   *   Yaw   (ψ): 绕 Z 轴转
   *
   * 从四元数到欧拉角的转换公式 (ZYX convention):
   *   φ = atan2( 2(q0q1 + q2q3),  1 - 2(q1² + q2²) )
   *   θ = asin(  2(q0q2 - q3q1)                    )   ← 需钳位 [-1,1]
   *   ψ = atan2( 2(q0q3 + q1q2),  1 - 2(q2² + q3²) )
   *
   * 输出单位: 度 (rad × 57.29578)
   * ═══════════════════════════════════════════════════════════════════════ */
  void MahonyGetEuler(float *roll, float *pitch, float *yaw)
  {
      /* Roll */
      *roll = atan2f(2.0f * (q0*q1 + q2*q3),
                     1.0f - 2.0f * (q1*q1 + q2*q2));

      /* Pitch — asin 的参数受浮点误差影响可能超 [-1, 1]，手动钳位 */
      float arg = 2.0f * (q0*q2 - q3*q1);
      if (arg >  1.0f) arg =  1.0f;
      if (arg < -1.0f) arg = -1.0f;
      *pitch = asinf(arg);

      /* Yaw */
      *yaw = atan2f(2.0f * (q0*q3 + q1*q2),
                    1.0f - 2.0f * (q2*q2 + q3*q3));

      /* 弧度 → 度: × 180 / π = × 57.29578 */
      const float r2d = 57.29578f;
      *roll  *= r2d;
      *pitch *= r2d;
      *yaw   *= r2d;
  }

  /* ═══════════════════════════════════════════════════════════════════════
   * MahonyGetRotationMatrix — 四元数 → 3×3 旋转矩阵
   *
   * 四元数 [w, x, y, z] 对应的旋转矩阵:
   *
   *       [ 1-2(y²+z²)   2(xy-wz)     2(xz+wy)  ]
   *   R = [ 2(xy+wz)     1-2(x²+z²)   2(yz-wx)  ]
   *       [ 2(xz-wy)     2(yz+wx)     1-2(x²+y²)]
   *
   * 存储方式: 行优先 (row-major)
   *   R[0] R[1] R[2]  = 第 0 行
   *   R[3] R[4] R[5]  = 第 1 行
   *   R[6] R[7] R[8]  = 第 2 行
   *
   * 用法: v_world = R × v_body
   *   aw_x = R[0]*ax + R[1]*ay + R[2]*az
   * ═══════════════════════════════════════════════════════════════════════ */
  void MahonyGetRotationMatrix(float R[3][3])
  {
      /* 预计算中间量，减少乘法次数 */
      float q1q1 = q1*q1;
      float q2q2 = q2*q2;
      float q3q3 = q3*q3;
      float q0q1 = q0*q1;
      float q0q2 = q0*q2;
      float q0q3 = q0*q3;
      float q1q2 = q1*q2;
      float q1q3 = q1*q3;
      float q2q3 = q2*q3;

      /* 第 0 行 */
      R[0][0] = 1.0f - 2.0f * (q2q2 + q3q3);   // 1-2(y²+z²)
      R[0][1] = 2.0f * (q1q2 - q0q3);          // 2(xy-wz)
      R[0][2] = 2.0f * (q1q3 + q0q2);          // 2(xz+wy)

      /* 第 1 行 */
      R[1][0] = 2.0f * (q1q2 + q0q3);          // 2(xy+wz)
      R[1][1] = 1.0f - 2.0f * (q1q1 + q3q3);   // 1-2(x²+z²)
      R[1][2] = 2.0f * (q2q3 - q0q1);          // 2(yz-wx)

      /* 第 2 行 */
      R[2][0] = 2.0f * (q1q3 - q0q2);          // 2(xz-wy)
      R[2][1] = 2.0f * (q2q3 + q0q1);          // 2(yz+wx)
      R[2][2] = 1.0f - 2.0f * (q1q1 + q2q2);   // 1-2(x²+y²)
  }