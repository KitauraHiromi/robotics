#include "../../lib/kinematics.hpp"
using namespace std;

int n, pn;
double pvar;
mt19937 mt;
uniform_real_distribution<double> th;

// mt19937 mt;
// uniform_real_distribution<double> th;


class line_path{
	public:
		pair<vd, vd> l;
		vector<vd> p;
		int n; //(=sizeof(points)
		double var;

		line_path(pair<vd, vd> _l, int _n, double _var){
			// param init
			this->n = _n;
			this->l = _l;
			this->var = _var;

		    for(int i=1; i<=_n; i++){
		    	vd p(3, 0);
		    	p[0] = (_l.second[0] - _l.first[0]) * i / (_n+1) + _l.first[0] + th(mt)*_var; // x
		    	p[1] = (_l.second[1] - _l.first[1]) * i / (_n+1) + _l.first[1] + th(mt)*_var; // y
		    	p[2] = (_l.second[2] - _l.first[2]) * i / (_n+1) + _l.first[2] + th(mt)*_var; // z
			    this->p.push_back(p);
			}
		}
		~line_path(){};
	};

vector<vd> yield_path(vector<line_path> lps){
	vector<vd> ret;
	size_t lps_n = lps.size();
	for(int i=0; i<lps_n; i++){
		ret.push_back(lps[i].l.first);
		for(int j=0; j<lps[i].n; j++){
			ret.push_back(lps[i].p[j]);		
		}
	}
	ret.push_back(lps[lps_n-1].l.second);
	return ret;
}


// generate tsp dataset by robot joint angle
// 1. XXXX
// Robot endefector travels point to point, while changing endefector posture randomly.
// Each point are connected with a line.
// In the middle of a line, middle points may be created and scattered, which number and variance is defined by parametera.
// parameters
// vector<vd> points
// struct line_path
//		parameters
// 			pair<vd, vd> line_path.line
//			vector<vd> points
//			int n (=sizeof(points))
//			double variance
//	methods
//		vector<vd> yield_path(vector<line_path>)
int main(){

    // init random
    std::random_device rd;
    mt = mt19937(rd());
    th = uniform_real_distribution<double>(-1, 1);

	// pn = 0;
	// pvar = 1;
	vector<vd> points;
	vector<line_path> lps;
    ofstream ofs("robot_pos.txt");
	cout << "n, pn, pvar" << endl;
	cin >> n >> pn >> pvar;

	// end efector position
	for(int i=0; i<n; i++){
		vd p(3, 0);
		cin >> p[0];
		cin >> p[1];
		cin >> p[2];
		points.push_back(p);
	}

	for(int i=0; i<n-1; i++){
		pair<vd, vd> l(points[i], points[i+1]);
		line_path lp(l, pn, pvar);
		lps.push_back(lp);
	}

	auto pos = yield_path(lps);

	// end efector posture
	double a, b, c;
	double pos_var = 0;
	a = 0; // base posture
	b = M_PI / 2;
	c = 0;
	cout << "x y z A B C" << endl;
	for(int i=0; i<pos.size(); i++){
		pos[i].push_back(a + th(mt)*pos_var);
		pos[i].push_back(b + th(mt)*pos_var);
		pos[i].push_back(c + th(mt)*pos_var);
		cout << pos[i][0] << ' ' << pos[i][1] << ' ' << pos[i][2] << ' ' << pos[i][3] << ' ' << pos[i][4] << ' ' << pos[i][5] << endl;
	}


	// define robot
	double l1, l2, l3; l1 = l2 = l3 = 1;
	dh puma = make_puma(l1, l2, l3);

	// ik
	for(int i=0; i<pos.size(); i++){
		tform p;
		p = pos_to_tform_xyz_eular(pos[i]);
		auto j = puma_ik(puma, p);
		ofs << j[0] << ' ' << j[1] << ' ' << j[2] << ' ' << j[3] << ' ' << j[4] << ' ' << j[5] << endl;
	}
	
	return 0;
}