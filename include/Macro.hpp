/*******************************************************************************
	Some efficient macros
*******************************************************************************/
#define SUM(_V_) std::accumulate(_V_.begin(), _V_.end(), 0);
//#define SUM(_V_, _F_) std::accumulate(_V_.begin(), _V_.end(), 0, _F_);

#define DIR_NOEXIST_AND_CREATE(_DIR_)	\
		system(("mkdir " + _DIR_).c_str())	

#define MAX_MIN(MAX, MIN, P, POS)	\
	if (MAX.POS < P.POS)	MAX.POS = P.POS;	\
	else if (MIN.POS > P.POS)	MIN.POS = P.POS;

#define Region_MAX_MIN(MAX, MIN, P)	\
	MAX_MIN(MAX, MIN, P, x)			\
	MAX_MIN(MAX, MIN, P, y)

#define elapsed_time  ((clock()- start)*1.0/CLOCKS_PER_SEC)

#define GetScreenSize	\
	ScreenHeight = ::GetSystemMetrics(SM_CXSCREEN);	\
	ScreenWidth = ::GetSystemMetrics(SM_CYSCREEN);

#define GetCameraSize(pxcinfo)	\
	CameraHeight = pxcinfo.height;	\
	CameraWidth = pxcinfo.width;

#define GetRatioXY	\
	RX = 1.0*ScreenHeight / CameraHeight;	\
	RY = 1.0*ScreenWidth / CameraWidth;

#define MESSAGE_WCOUT(_TYPE_,_INFO_)	\
	std::wcout << "[" << _TYPE_ << "]\t" << _INFO_ << std::endl

#define MESSAGE_COUT(_TYPE_,_INFO_)	\
	std::cout << "[" << _TYPE_ << "]\t" << _INFO_ << std::endl

#define POINT_CVT(P)	\
	P.x ^= P.y;			\
	P.y ^= P.x;			\
	P.x ^= P.y;

#define DIM1TODIM2(I,J,V,ROW)	V[I*ROW+J] 
#define BETWEEN(VAL,LOW,UP)	((VAL)>(LOW)&&(VAL)<(UP))

//The point at the center of region
#define MiddlePoint(RECT)	\
	Point(RECT.x + (RECT.width >> 1), RECT.y + (RECT.height >> 1))

/*******************************************************************************
Some configuration
*******************************************************************************/
#define GRASP_START "启动程序！"
#define GRASP_REG_SUCCESS "配准成功！"
#define GRASP_ACTION "实施抓取！"

#define COLOR_MODEL Vec3b(255, 255, 0)
#define COLOR_GRASP Vec3b(0, 255, 255)