/*默认使用功率、爆发优先*/

/*机器人编号*/
/*1. 英雄机器人
  2. 工程机器人
  3. 步兵机器人
  4. 步兵机器人
  5. 哨兵机器人*/
#define Robot_ID 3//填写机器人编号激活对应定义

/*电机数量*/
#define M3508_NUM 
#define M2006_NUM
#define GM6020_NUM

/*机器人底盘类型*/
#define CHASSIS_TYPE 'OMNI'

/*步兵底盘优先类型*/
#define POWER_PRIORITY //功率优先
//#define BLOOD_PRIORITY //血量优先

/*发射机构类型*/
#define BURST //爆发优先
//#define COOLING //冷却优先

/*英雄机器人属性*/
#if (Robot_ID == 1)
#define POWER_MAX 100
#define HEAT_MAX 200
#define COOLING_SPEED 24 //Unit: J/s
#define BLOOD_MAX 300
#define BULLET_SPEED_MAX 12
#endif

/*步兵机器人属性*/
#if ((Robot_ID == 3) || (Robot_ID == 4))
#define BULLET_SPEED_MAX 25
#endif

#if ((Robot_ID == 3) || (Robot_ID == 4)) && defined (POWER_PRIORITY)
#define POWER_MAX 90
#define BLOOD_MAX 300
#endif

#if ((Robot_ID == 3) || (Robot_ID == 4)) && defined (BLOOD_PRIORITY)
#define POWER_MAX 75
#define BLOOD_MAX 350
#endif

#if ((Robot_ID == 3) || (Robot_ID == 4)) && defined (BURST)
#define HEAT_MAX 230
#define COOLING_SPEED 14
#endif

#if ((Robot_ID == 3) || (Robot_ID == 4)) && defined (COOLING)
#define HEAT_MAX 88
#define COOLING_SPEED 24
#endif

/*哨兵机器人属性*/
#if (Robot_ID == 5)
#define POWER_MAX 100
#define HEAT_MAX 230
#define COOLING_SPEED 30
#define BLOOD_MAX 400
#define BULLET_SPEED_MAX 25
#endif
