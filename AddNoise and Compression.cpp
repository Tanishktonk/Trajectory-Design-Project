/*
 * 
 * 
 * Author: Tanishk <tanishktonk>
 * Time: 17.11.2022 14:50:58 IST
 * ........ ;)
 * 
 * 
 */

// #pragma GCC optimize("Ofast")
// #pragma GCC target("avx,avx2,fma")
// #pragma GCC optimization ("unroll-loops") 
#include <bits/stdc++.h>
#define ll long long int
#define fr(i,n) for(i=0;i<n;i++)
#define rng(i,a,b) for(i=a;i<b;i++)
#define T long long int T;cin>>T;while(T--)
#define ain(a,n) for(int zzz=0;zzz<n;zzz++){cin>>a[zzz];}
#define aout(a,n) for(int zzz=0;zzz<n;zzz++){cout<<a[zzz]<<' ';}cout<<"\n";
#define All(v) v.begin(),v.end()
#define dbg(v) cerr<<">-- "<<#v<<" = "<<v<<"\n";
#define PB push_back
#define MP make_pair
#define N "\n"
using namespace std;

// debug template
#define sim template < class c
#define ris return * this
#define dor > debug & operator <<
#define eni(x) sim > typename \
enable_if<sizeof dud<c>(0) x 1, debug&>::type operator<<(c zzz) {
sim > struct rge { c b, e; };
sim > rge<c> range(c zzz, c yyy) { return rge<c>{zzz, yyy}; }
sim > auto dud(c* x) -> decltype(cerr << *x, 0);
sim > char dud(...);
struct debug {
#ifdef LOCAL
~debug() { cerr << endl; }
eni(!=) cerr << boolalpha << zzz; ris; }
eni(==) ris << range(begin(zzz), end(zzz)); }
sim, class b dor(pair < b, c > d) {
  ris << "(" << d.first << ", " << d.second << ")";
}
sim dor(rge<c> d) {
  *this << "[";
  for (auto it = d.b; it != d.e; ++it)
	*this << ", " + 2 * (it == d.b) << *it;
  ris << "]";
}
#else
sim dor(const c&) { ris; }
#endif
};
#define imie(...) " [" << #__VA_ARGS__ ": " << (__VA_ARGS__) << "] "

typedef long double ld;			const ld pi = 3.14159265358979323846;
typedef vector<int> Vi;			const ld scale = 1e9;
typedef vector<ll> Vl;			const ld eps = 1e-9;
typedef pair<int,int> Pii;		const int inf = 1e9+3;
typedef pair<ll,ll> Pll;		const ll linf = 1e18+3;
typedef pair<int,Pii> Piii;		const int M97 = 1e9+7;

struct point
{
	ld lat, lng;
	int id, uid;
	string time;
};

void input(vector<point> &points)  // takes input from "mempool.csv" file
{
	ifstream inp("go_track_trackspoints.csv", ios::in);
	string record,data;
	getline(inp,record);
	
	while(getline(inp,record))
	{
		point x;
		stringstream s(record);
		
		getline(s,data,',');
		x.id = stoi(data);
		
		getline(s,data,',');
		x.lat = stold(data);
		
		getline(s,data,',');
		x.lng = stold(data);
		
		getline(s,data,',');
		x.uid = stoi(data);
		
		getline(s,data);
		x.time = data;
		
		points.emplace_back(x);
	}
	
	inp.close();
}

void output(vector<point> &points)
{
	cout << "\"id\",\"latitude\",\"longitude\",\"track_id\",\"time\"\n";
	for(point &i: points)
		cout << i.id << "," << i.lat << "," << i.lng << "," << i.uid << "," << i.time << N;
}

void addNoise(vector<point> &points)
{
	ld mnlat = -90, mxlat = 90;
	ld mnlng = -180, mxlng = 180;
	srand(time(0));
	
	for(point& x: points)
		if(rand() % 30 == 0)
		{
			ld eps = 1.0/(600.0+(rand() % 500));
			int choice = rand() % 4;
			int split = rand() % 11;
			if(choice == 0)
				x.lat += (eps*split/10.0),
				x.lng += (eps*(10-split)/10.0);
			else if(choice == 1)
				x.lat -= (eps*split/10.0),
				x.lng += (eps*(10-split)/10.0);
			else if(choice == 2)
				x.lat += (eps*split/10.0),
				x.lng -= (eps*(10-split)/10.0);
			else
				x.lat -= (eps*split/10.0),
				x.lng -= (eps*(10-split)/10.0);
				
			if(x.lat > mxlat)
				x.lat = mxlat;
			if(x.lat < mnlat)
				x.lat = mnlat;
			if(x.lng < mnlng)
				x.lng = mnlng;
			if(x.lng > mxlng)
				x.lng = mxlng;
		}
}

ld perpEuclidDist(point &l1, point &l2, point &p)
{
	ld m = (l2.lng - l1.lng) / (l2.lat - l1.lat);
	ld a = m, b = -1, c = l1.lng - m * l1.lat;
	ld dist = abs(a * p.lat + b * p.lng + c) / sqrtl(a * a + b * b);
	// debug() << dist;
	return dist;
}

void compressTraj(vector<point> &points, ld &thres, int l, int r, ld &totErr, ld &maxErr, vector<point> &compressed)
{
	if(l == r)
		return;
	ld mxDist = 0, tot = 0;
	int ind = -1;
	
	for(int i = l+1; i < r; i++)
	{
		ld d = perpEuclidDist(points[l], points[r], points[i]);
		tot += d;
		if(mxDist < d)
		{
			mxDist = d;
			ind = i;
		}
	}
	
	if(mxDist < thres || ind == -1)
	{
		totErr += tot;
		maxErr = max(maxErr, mxDist);
		compressed.push_back(points[l]);
		return;
	}
	
	compressTraj(points, thres, l, ind, totErr, maxErr, compressed);
	compressTraj(points, thres, ind, r, totErr, maxErr, compressed);
}

ll i,j,k;

void solve()
{
	vector<point> points;
	input(points);
	
	vector<point> cp;
	ld thres = 0.00001;
	for(int i = 0; i < (int)points.size(); i++)
	{
		if(points[i].uid != 208)
			continue;
		int j = i+1;
		while(j < (int)points.size() && points[j].uid == points[i].uid) j++;
		j--;
		
		vector<point> compressed;
		ld totErr = 0, maxErr = 0;
		compressTraj(points, thres, i, j, totErr, maxErr, compressed);
		// debug() << j-i+1 << ' ' << compressed.size();
		compressed.push_back(points[j]);
		ld compressPercent = 100 - (((ld)compressed.size()) * 100.0 / (j-i+1.0));
		
		cout << "User = " << points[i].uid << N;
		cout << "Initial and Final no. of points = " << j-i+1 << ' ' << compressed.size() << N;
		cout << "Max Error = " << maxErr << N;
		cout << "Total Error = " << totErr << N;
		cout << "Compression % = " << compressPercent << N << N;
		cp.insert(cp.end(), compressed.begin(), compressed.end());
		
		i = j;
	}
	
	
	// output(cp);
}

int main()
{
	ios_base::sync_with_stdio(false);
	cin.tie(NULL);
	
	cout << setprecision(14);
	solve();
	
	return 0;
}
