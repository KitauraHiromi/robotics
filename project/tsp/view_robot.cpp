#include "../../lib/glutil/glutil.hpp"
#include "../../lib/kinematics.hpp"
using namespace std;

double l1 = 1;
double l2 = 1;
double l3 = 1;
double pointsize = 20;
double linewidth = 15;
vector<vd> pos;
vector<vector<vd>> robot_view;
dh puma;
double m = 50;
int view = 0;
int max_view;

void draw_robot(vector<vector<vd>> robot_view){
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// colorとlightは共存できない。今回はcolorで色を付ける。
	glColor3f(1, 0, 0);
	glBegin(GL_POINTS);
	for(vd p: robot_view[view]) glVertex3f(m*p[0], m*p[1], m*p[2]);
	glEnd();

	glBegin(GL_LINE_STRIP);
	glColor3f(1, 1, 1);
	for(vd p: robot_view[view]) glVertex3f(m*p[0], m*p[1], m*p[2]);
	glEnd();
}

void line3D(double x1,double y1,double z1,double x2,double y2,double z2){
	//線幅
	glLineWidth(1.0);
	//線
	glBegin(GL_LINES);
	glVertex3f(x1, y1, z1);
	glVertex3f(x2, y2, z2);
	glEnd();
}

void draw_measure(int measure,double size){
	// draw grid
	glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
	for(int x=0; x<=measure; x++){
		line3D(x*size-(size*measure/2), 0, -(size*measure/2), x*size-(size*measure/2), 0, measure*size-(size*measure/2));
	}
	for(int y=0; y<=measure; y++){
		line3D(-(size*measure/2), 0, y*size-(size*measure/2), measure*size-(size*measure/2), 0, y*size-(size*measure/2));
	}
	// draw x axis
	glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
	line3D(0,0,0,(measure/2+2)*size,0,0);
	// draw y axis
	glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
	line3D(0,0,0,0,(measure/2+2)*size,0);
	// draw z axis
	glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
	line3D(0,0,0,0,0,(measure/2+2)*size);
}

vector<vd> get_robot_view(dh link, vd theta){
	vector<vd> ret;
	tform link_end_pos = tform::Identity();
	vd origin({link_end_pos(0, 3), link_end_pos(1, 3), link_end_pos(2, 3)});
	ret.push_back(origin);
	for(int i=0; i<link.n; i++){
		link_end_pos *= link.h(link.param[i], theta[i]);
		vd link_end({link_end_pos(0, 3), link_end_pos(1, 3), link_end_pos(2, 3)});
		ret.push_back(link_end);
	}
	return ret;
}


void display(void){
	glViewport(0, 0, WIDTH, HEIGHT);
	glMatrixMode(GL_PROJECTION);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA | GLUT_DEPTH);

	glLoadIdentity();
	//視野角,アスペクト比(ウィンドウの幅/高さ),描画する範囲(最も近い距離,最も遠い距離)
	gluPerspective(30.0, (double)WIDTH / (double)HEIGHT, 1.0, 1000.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//視点の設定
	gluLookAt(150.0,100.0,-200.0, //カメラの座標
			  0.0,0.0,0.0, // 注視点の座標
			  0.0,1.0,0.0); // 画面の上方向を指すベクトル
	//クォータニオンによる回転
	glMultMatrixd(Rotate);
	glPointSize(pointsize);
	glLineWidth(linewidth);

	// ロボットの描画
	draw_robot(robot_view);
	// グリッドの描画
	draw_measure(100, 10);
	glutSwapBuffers();
}

void key(unsigned char key, int x, int y){
	if(key == 'z') view++;
	if(view >= max_view) view = 0;
}

int main(int argc, char *argv[]){
	string str;
	ifstream ifs("robot_pos.txt");
	while(getline(ifs, str)){
		vd j(6, 0);
		istringstream iss(str);
		iss >> j[0] >> j[1] >> j[2] >> j[3] >> j[4] >> j[5];
		pos.push_back(j);
	}

	puma = make_puma(l1, l2, l3);
	max_view = pos.size();
	for(vd j: pos)
		robot_view.push_back(get_robot_view(puma, j));

	glutInitWindowPosition(100, 100);
	glutInitWindowSize(WIDTH, HEIGHT);
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA);
	glutCreateWindow("puma型ロボット");
	glutDisplayFunc(display);
	glutMouseFunc(mouse);
	glutMotionFunc(mousemove);
	glutKeyboardFunc(key);
	glutIdleFunc(idle);
	Init();
	glutMainLoop();
	return 0;
}