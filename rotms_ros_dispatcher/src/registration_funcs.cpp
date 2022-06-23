#include "registration_funcs.hpp"

#include <vector>
#include <tuple>
#include <time.h>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <math.h>
#include <limits>

std::vector<double> rotm2quat(const Matrixnbym& R)
{
    double qw = sqrt(1.0+R[0][0]+R[1][1]+R[2][2]) / 2.0 * 4.0;
    // x y z w
    std::vector<double> ans{ 
        (R[2][1] - R[1][2]) / qw,
        (R[0][2] - R[2][0]) / qw,
        (R[1][0] - R[0][1]) / qw,
        qw/4
    };
    return ans;
}

void SaveRegistrationData(
    const std::vector<double>& quat, const std::vector<double>& p, const std::string& f)
{
	std::ofstream filesave(f);
	if(filesave.is_open())
	{
        filesave << "TRANSLATION: # translation: x,y,z\n";
        filesave << "\n";
        filesave << "  {\n";
		filesave << "    x: " << FormatDouble2String(p[0], 16) << ",\n";
		filesave << "    y: " << FormatDouble2String(p[1], 16) << ",\n";
		filesave << "    z: " << FormatDouble2String(p[2], 16) << "\n";
		filesave << "  }\n";
        filesave << "\n";
		filesave << "ROTATION: # quat: x,y,z,w\n";
        filesave << "\n";
        filesave << "  {\n";
		filesave << "    x: " << FormatDouble2String(quat[0], 16) << ",\n";
		filesave << "    y: " << FormatDouble2String(quat[1], 16) << ",\n";
		filesave << "    z: " << FormatDouble2String(quat[2], 16) << ",\n";
        filesave << "    w: " << FormatDouble2String(quat[3], 16) << "\n";
        filesave << "  }\n";
		filesave.close();
	}
}

std::string FormatDouble2String(double a, int dec)
{
	std::stringstream stream;
    stream << std::fixed << std::setprecision(dec) << a;
    std::string s = stream.str();
    return s;
}

std::string GetTimeString()
{
	time_t rawtime;
	struct tm * timeinfo;
	char buffer [80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime (buffer,80,"%Y%m%d_%I%M%p",timeinfo);

	return buffer;
}

std::tuple<Matrixnbym, std::vector<double>> getRegistrationResult(
    const Matrixnbym& a, const Matrixnbym& b)
{
    int sz1 = a.size();
    int sz2 = b.size();

    if(sz1!=sz2)
        throw std::runtime_error(
            "Planned point cloud and the digitized point cloud have different size!");

    std::vector<double> a_centroid = getCentroid(a);
	std::vector<double> b_centroid = getCentroid(b);
	Matrixnbym A_tilda = getDeviations(a, a_centroid);
	Matrixnbym B_tilda = getDeviations(b, b_centroid);
	Matrixnbym H = getZeros3by3();
	for(int i=0;i<sz1;i++)
	{
		H[0][0] += A_tilda[i][0]*B_tilda[i][0];
		H[0][1] += A_tilda[i][0]*B_tilda[i][1];
		H[0][2] += A_tilda[i][0]*B_tilda[i][2];
		H[1][0] += A_tilda[i][1]*B_tilda[i][0];
		H[1][1] += A_tilda[i][1]*B_tilda[i][1];
		H[1][2] += A_tilda[i][1]*B_tilda[i][2];
		H[2][0] += A_tilda[i][2]*B_tilda[i][0]; 
		H[2][1] += A_tilda[i][2]*B_tilda[i][1];
		H[2][2] += A_tilda[i][2]*B_tilda[i][2];
	}
    
	std::tuple<Matrixnbym, Matrixnbym, Matrixnbym> USV = svdSim3by3(H);
	Matrixnbym R = matrixMult3by3(std::get<2>(USV), transpose3by3(std::get<0>(USV)));
    
	if(det3by3(R) < 0)
	{
		Matrixnbym V = std::get<2>(USV);
		V[0][0] = -V[0][0]; V[0][1] = -V[0][1]; V[0][2] = -V[0][2];
        V[1][0] = -V[1][0]; V[1][1] = -V[1][1]; V[1][2] = -V[1][2];
        V[2][0] = -V[2][0]; V[2][1] = -V[2][1]; V[2][2] = -V[2][2];

        R = matrixMult3by3(V, transpose3by3(std::get<0>(USV)));
	}
    
	std::vector<double> p{0.0,0.0,0.0};
	p[0] = b_centroid[0] - (
	    R[0][0] * a_centroid[0] + R[0][1] * a_centroid[1] + R[0][2] * a_centroid[2]);
	p[1] = b_centroid[1] - (
	    R[1][0] * a_centroid[0] + R[1][1] * a_centroid[1] + R[1][2] * a_centroid[2]);
	p[2] = b_centroid[2] - (
	    R[2][0] * a_centroid[0] + R[2][1] * a_centroid[1] + R[2][2] * a_centroid[2]);

    return std::make_tuple(R, p);
}

Matrixnbym getZeros3by3()
{
	std::vector<double> vec1{0.0, 0.0, 0.0};
	std::vector<double> vec2{0.0, 0.0, 0.0};
	std::vector<double> vec3{0.0, 0.0, 0.0};
	Matrixnbym H{vec1, vec2, vec3};
	return H;
}

Matrixnbym getZeros(int n)
{
	Matrixnbym I;
    for(int i = 0; i < n; i++)
    {
    	std::vector<double> vec{ 0.0f, 0.0f, 0.0f };
        I.push_back(vec);
    }
    return I;
}

Matrixnbym getEye3by3()
{
	std::vector<double> vec1{1.0, 0.0, 0.0};
	std::vector<double> vec2{0.0, 1.0, 0.0};
	std::vector<double> vec3{0.0, 0.0, 1.0};
	Matrixnbym H{vec1, vec2, vec3};
	return H;
}

Matrixnbym getEye(int n)
{
    Matrixnbym H;
    for(int i=0;i<n;i++){
        std::vector<double> temp;
        for(int j=0;j<n;j++){
            if(i==j)
                temp.push_back(1.0);
            else
                temp.push_back(0.0);
        }
        H.push_back(temp);
    }
	return H;
}

Matrixnbym getDiag3by3(const std::vector<double>& v)
{
	Matrixnbym D = getZeros3by3();
	D[0][0] = v[0];
    D[1][1] = v[1];
    D[2][2] = v[2];
    return D;
}

Matrixnbym matrixMult3by3(const Matrixnbym& X, const Matrixnbym& Y)
{
	Matrixnbym A = getZeros3by3();
	for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                A[i][j] += X[i][k] * Y[k][j];
    return A;
}

Matrixnbym matrixMult(const Matrixnbym& X, const Matrixnbym& Y)
{
	Matrixnbym A = getZeros(X.size());
	for (int i = 0; i < X.size(); i++)
        for (int j = 0; j < X.size(); j++)
            for (int k = 0; k < X.size(); k++)
                A[i][j] += X[i][k] * Y[k][j];
    return A;
}

Matrixnbym transpose3by3(const Matrixnbym& A)
{
	Matrixnbym H = getZeros3by3();
	for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            H[j][i] = A[i][j];
    return H;
}

double det3by3(const Matrixnbym& R)
{
	return R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]) - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0]) + R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1]);
}

std::vector<double> getCentroid(const Matrixnbym& A)
{
	int n = A.size();
	std::vector<double> aCentroid{0.0,0.0,0.0};
	for (int i = 0; i < n; i++)
    {
        aCentroid[0] += A[i][0];
        aCentroid[1] += A[i][1];
        aCentroid[2] += A[i][2];
    }
    aCentroid[0] = (1.0 / n) * aCentroid[0];
    aCentroid[1] = (1.0 / n) * aCentroid[1];
    aCentroid[2] = (1.0 / n) * aCentroid[2];
    return aCentroid;
} 

Matrixnbym getDeviations(const Matrixnbym& A, const std::vector<double>& aCentroid)
{
	int n = A.size();
	Matrixnbym aTilda = getZeros(n);
    for (int i = 0; i < n; i++)
    {
        aTilda[i][0] = A[i][0] - aCentroid[0];
        aTilda[i][1] = A[i][1] - aCentroid[1];
        aTilda[i][2] = A[i][2] - aCentroid[2];
    }
    return aTilda;
}

// https://rosettacode.org/wiki/QR_decomposition
std::tuple<Matrixnbym, Matrixnbym> qrSim3by3(const Matrixnbym& A)
{
    Matrixnbym R = getZeros3by3();
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            R[i][j] = A[i][j];
        }
    }
    Matrixnbym Q = getEye3by3();
    for(int i=0;i<2;i++)
    {
        Matrixnbym H = getEye3by3();
        std::vector<double> a;
        for(int j=i;j<3;j++){
            a.push_back(R[j][i]);
        }
        Matrixnbym H_ = qrhelper(a);
        
        for(int j=i;j<3;j++){
            for(int q=i;q<3;q++){
                H[j][q] = H_[j-i][q-i];
            }
        }
        
        Q = matrixMult(Q,H);
        R = matrixMult(H,R);
        
    }

	return std::make_tuple(Q, R);

}

Matrixnbym qrhelper(const std::vector<double>& a)
{
    double a_norm = 0.0;
    for(int i=0;i<a.size();i++){
        a_norm += a[i] * a[i];
    }
    a_norm = sqrt(a_norm);
    
    std::vector<double> v;
    for(int i=0;i<a.size();i++){
        v.push_back(a[i] / (a[0] + std::copysign(a_norm,a[0])));
    }
    v[0] = 1.0;
    Matrixnbym H = getEye(a.size());
    Matrixnbym V;
    for(int i=0;i<v.size();i++){
        std::vector<double> temp;
        for(int j=0;j<v.size();j++){
            temp.push_back(v[i]*v[j]);
        }
        V.push_back(temp);
    }
    

    double v_norm = 0.0;
    for(int i=0;i<v.size();i++){
        v_norm += v[i] * v[i];
    }
    for(int i=0;i<v.size();i++){
        for(int j=0;j<v.size();j++){
            H[i][j] -= 2.0 / v_norm * V[i][j];
        }
    }
    return H;
}

std::tuple<Matrixnbym, Matrixnbym, Matrixnbym> svdSim3by3(const Matrixnbym& A)
{
	Matrixnbym U;
	Matrixnbym S;
	Matrixnbym V;
	Matrixnbym Q;

	double tol = 2.2204E-16f * 1024;  // eps floating-point relative accuracy 
	int loopmax = 300;
    int loopcount = 0;

    U = getEye3by3();
    V = getEye3by3();
    S = transpose3by3(A);

    double err = std::numeric_limits<float>::max();

    while (err > tol && loopcount < loopmax)
    {
    	std::tuple<Matrixnbym, Matrixnbym> tulp = qrSim3by3(transpose3by3(S));
    	Q = std::get<0>(tulp);
    	S = std::get<1>(tulp);
    	U = matrixMult3by3(U, Q);

    	tulp = qrSim3by3(transpose3by3(S));
    	Q = std::get<0>(tulp);
    	S = std::get<1>(tulp);
    	V = matrixMult3by3(V, Q);

    	Matrixnbym e = getZeros3by3();
    	e[0][1] = S[0][1];
        e[0][2] = S[0][2];
        e[1][2] = S[1][2];

        double E = (double)sqrt(e[0][1] * e[0][1] + e[0][2] * e[0][2] + e[1][2] * e[1][2]);
        double F = (double)sqrt(S[0][0] * S[0][0] + S[1][1] * S[1][1] + S[2][2] * S[2][2]);

        if (F == 0)
            F = 1;
        err = E / F;
        loopcount++;

    }

    std::vector<double> SS{S[0][0], S[1][1], S[2][2]};

    for (int i = 0; i < 3; i++)
    {
        double SSi = abs(SS[i]);
        S[i][i] = SSi;
        if (SS[i] < 0)
        {
            U[0][i] = -U[0][i];
            U[1][i] = -U[1][i];
            U[2][i] = -U[2][i];
        }
    }

    return std::make_tuple(U, S, V);
}