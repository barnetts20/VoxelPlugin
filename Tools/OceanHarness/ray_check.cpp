// O3b: the underwater pass's ray from the depth buffer, in the shader's float arithmetic, against
// double truth. UE conventions: row vectors, reversed-z infinite perspective with TAA jitter,
// view space x right / y up / z forward. Also checks the per-view skip (the near plane's reach).
#include <cmath>
#include <cstdio>
#include <random>
#include <algorithm>
typedef double M4[4][4];
static void Mul(const M4 A, const M4 B, M4 C){ for(int i=0;i<4;i++)for(int j=0;j<4;j++){double s=0;for(int k=0;k<4;k++)s+=A[i][k]*B[k][j];C[i][j]=s;} }
static void VecMul(const double v[4], const M4 M, double o[4]){ for(int j=0;j<4;j++){o[j]=0;for(int k=0;k<4;k++)o[j]+=v[k]*M[k][j];} }
static void VecMulF(const float v[4], const float M[4][4], float o[4]){ for(int j=0;j<4;j++){o[j]=0;for(int k=0;k<4;k++)o[j]+=v[k]*M[k][j];} }
static bool Inverse(const M4 m, M4 inv){ // Gauss-Jordan
  double a[4][8]; for(int i=0;i<4;i++){for(int j=0;j<4;j++){a[i][j]=m[i][j];a[i][j+4]=i==j;}}
  for(int c=0;c<4;c++){int p=c;for(int r=c+1;r<4;r++) if(fabs(a[r][c])>fabs(a[p][c]))p=r; if(fabs(a[p][c])<1e-300) return false;
    for(int j=0;j<8;j++) std::swap(a[c][j],a[p][j]); double d=a[c][c]; for(int j=0;j<8;j++) a[c][j]/=d;
    for(int r=0;r<4;r++) if(r!=c){double f=a[r][c]; for(int j=0;j<8;j++) a[r][j]-=f*a[c][j];}}
  for(int i=0;i<4;i++)for(int j=0;j<4;j++)inv[i][j]=a[i][j+4]; return true; }
int main(){
  std::mt19937_64 rng(7); std::uniform_real_distribution<double> U(0,1);
  double maxNearErr=0, maxDirErr=0, maxDistRel=0, maxDistRelNear=0; long reachFail=0, n=0, skyOk=0, skyN=0;
  for(int cam=0; cam<400; cam++){
    const double halfFov = (30+U(rng)*45)*M_PI/180, W=1920, H=800+U(rng)*600, MinZ=[&]{double r=U(rng);return r<0.5?10.0:(r<0.8?1.0:30.0);}();
    const double jx=(U(rng)-0.5)*2/W, jy=(U(rng)-0.5)*2/H;
    M4 P={{1/tan(halfFov),0,0,0},{0,W/tan(halfFov)/H,0,0},{jx,jy,0,1},{0,0,MinZ,0}};
    // random rotation (world->view), orthonormal
    double q[4]; double s=0; for(int i=0;i<4;i++){q[i]=U(rng)*2-1;s+=q[i]*q[i];} s=sqrt(s); for(int i=0;i<4;i++)q[i]/=s;
    double w=q[0],x=q[1],y=q[2],z=q[3];
    M4 R={{1-2*(y*y+z*z),2*(x*y+w*z),2*(x*z-w*y),0},{2*(x*y-w*z),1-2*(x*x+z*z),2*(y*z+w*x),0},{2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y),0},{0,0,0,1}};
    M4 Pinv, Rinv, C; Inverse(P,Pinv); for(int i=0;i<4;i++)for(int j=0;j<4;j++)Rinv[i][j]=R[j][i];
    Mul(Pinv,Rinv,C);              // ClipToCamera = InvProjection * InvView.RemoveTranslation
    float Cf[4][4]; for(int i=0;i<4;i++)for(int j=0;j<4;j++)Cf[i][j]=(float)C[i][j];
    double reach=0; for(int k=0;k<4;k++){double v[4]={(k&1)?1.:-1.,(k&2)?1.:-1.,1,1},h[4];VecMul(v,C,h); if(h[3]>0) reach=std::max(reach,sqrt(h[0]*h[0]+h[1]*h[1]+h[2]*h[2])/h[3]);}
    for(int px=0; px<2000; px++){
      // pixel centre in a W x H view
      const int ix=(int)(U(rng)*W), iy=(int)(U(rng)*H);
      const float u=((float)ix+0.5f)/(float)W, vv=((float)iy+0.5f)/(float)H;
      const float cx=2*u-1, cy=1-2*vv;
      // truth: the ray through this clip point (double)
      double nd[4]={cx,cy,1,1},nh[4]; VecMul(nd,C,nh); double nt[3]={nh[0]/nh[3],nh[1]/nh[3],nh[2]/nh[3]};
      double nlen=sqrt(nt[0]*nt[0]+nt[1]*nt[1]+nt[2]*nt[2]); double dir[3]={nt[0]/nlen,nt[1]/nlen,nt[2]/nlen};
      if(nlen>reach*(1+1e-12)) reachFail++;
      // a scene point along it at a random distance 0.2 m .. 200 km, its device z in float (as the depth buffer stores it)
      const double dist = nlen*pow(10.0, U(rng)*6.5);
      double pw[4]={dir[0]*dist,dir[1]*dist,dir[2]*dist,1}, pv[4], pc[4]; VecMul(pw,R,pv); VecMul(pv,P,pc);
      const float devz=(float)(pc[2]/pc[3]);
      // the true distance of the point the stored device z stands for (depth quantised to float)
      double sd[4]={cx,cy,(double)devz,1},sh[4]; VecMul(sd,C,sh); const double distQ=sqrt(sh[0]*sh[0]+sh[1]*sh[1]+sh[2]*sh[2])/sh[3];
      // shader float
      float a[4]={cx,cy,1.f,1.f},NH[4]; VecMulF(a,Cf,NH);
      float b[4]={cx,cy,devz,1.f},SH[4]; VecMulF(b,Cf,SH);
      float NL=std::max(sqrtf(NH[0]*NH[0]+NH[1]*NH[1]+NH[2]*NH[2]),1e-20f);
      float D[3]={NH[0]/NL,NH[1]/NL,NH[2]/NL}; float ND=NL/std::max(NH[3],1e-20f);
      float SD= SH[3]>0? sqrtf(SH[0]*SH[0]+SH[1]*SH[1]+SH[2]*SH[2])/SH[3] : 1e30f;
      maxNearErr=std::max(maxNearErr,fabs(ND-nlen));
      double de=sqrt((D[0]-dir[0])*(D[0]-dir[0])+(D[1]-dir[1])*(D[1]-dir[1])+(D[2]-dir[2])*(D[2]-dir[2]));
      maxDirErr=std::max(maxDirErr,de);
      double rel=fabs(SD-distQ)/distQ; maxDistRel=std::max(maxDistRel,rel); if(dist<1e4) maxDistRelNear=std::max(maxDistRelNear,rel);
      n++;
      // sky: device z 0
      float c0[4]={cx,cy,0.f,1.f},S0[4]; VecMulF(c0,Cf,S0); skyN++; if(!(S0[3]>0)) skyOk++;
    }
  }
  printf("rays %ld: near-plane distance max error %.3g cm, direction max error %.3g rad, scene distance max rel error %.3g (%.3g within 100 m) against the stored depth\n",n,maxNearErr,maxDirErr,maxDistRel,maxDistRelNear);
  printf("near points beyond the corner reach: %ld; sky pixels with no finite point: %ld of %ld\n",reachFail,skyOk,skyN);
}
