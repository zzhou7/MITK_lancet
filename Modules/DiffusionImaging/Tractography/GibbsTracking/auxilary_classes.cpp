/*===================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center,
Division of Medical and Biological Informatics.
All rights reserved.

This software is distributed WITHOUT ANY WARRANTY; without
even the implied warranty of MERCHANTABILITY or FITNESS FOR
A PARTICULAR PURPOSE.

See LICENSE.txt or http://www.mitk.org for details.

===================================================================*/

#ifndef _AUXCLASS
#define _AUXCLASS

#ifndef INFINITY
#define INFINITY 1000000000
#endif

#include "MersenneTwister.h"
#include <stdlib.h>
using namespace std;

MTRand mtrand;

class pVector
{
private:
    float x;
    float y;
    float z;

public:

    pVector()
    {

    }

    pVector(float x,float y,float z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    inline void SetXYZ(float sx,float sy, float sz)
    {
        x = sx;
        y = sy;
        z = sz;
    }

    inline void GetXYZ(float *xyz)
    {
        xyz[0] = x;
        xyz[1] = y;
        xyz[2] = z;
    }

    inline float GetX()
    {
        return x;
    }

    inline float GetY()
    {
        return y;
    }

    inline float GetZ()
    {
        return z;
    }

    inline void rand(float w, float h, float d)
    {
        this->x = mtrand.frand()*w;
        this->y = mtrand.frand()*h;
        this->z = mtrand.frand()*d;
    }

    inline void rand_sphere()
    {
        this->x = mtrand.frandn();
        this->y = mtrand.frandn();
        this->z = mtrand.frandn();
        normalize();
    }

    inline void normalize()
    {
        float norm = sqrt(x*x+y*y+z*z)+ 0.00000001;
        *this /= norm;
    }

    inline float norm_square()
    {
        return x*x + y*y + z*z;
    }

    inline void distortn(float sigma)
    {
        x += sigma*mtrand.frandn();
        y += sigma*mtrand.frandn();
        z += sigma*mtrand.frandn();
    }

    inline float operator[](int index)
    {
        switch(index)
        {
        case 0:
            return x;
        case 1:
            return y;
        case 2:
            return z;
        default:
            return 0.0f;
        }
    }

    inline pVector operator*(float s)
    {
        return pVector(s*x,s*y,s*z);
    }

    inline void operator*=(float &s)
    {
        x *= s;
        y *= s;
        z *= s;
    }

    inline pVector operator+(pVector R)
    {
        return pVector(x+R.x,y+R.y,z+R.z);
    }

    inline void operator+=(pVector R)
    {
        x += R.x;
        y += R.y;
        z += R.z;
    }

    inline pVector operator-(pVector R)
    {
        return pVector(x-R.x,y-R.y,z-R.z);
    }

    inline void operator-=(pVector R)
    {
        x -= R.x;
        y -= R.y;
        z -= R.z;
    }

    inline pVector operator/(float &s)
    {
        return pVector(x/s,y/s,z/s);
    }

    inline void operator/=(float &s)
    {
        x /= s;
        y /= s;
        z /= s;
    }


    inline float operator*(pVector R)
    {
        return x*R.x+y*R.y+z*R.z;
    }
};

class Particle
{
public:

    Particle()
    {
        label = 0;
        pID = -1;
        mID = -1;
        inserted = false;
    }

    ~Particle()
    {
    }

    pVector R;
    pVector N;
    float cap;
    float len;

    int gridindex; // index in the grid where it is living
    int ID;
    int pID;
    int mID;

    int label;
    int numerator;
    bool inserted;
};


class EnergyGradient
{
public:
    pVector gR;
    pVector gN;

    inline float norm2()
    {
        return gR.norm_square() + gN.norm_square();
    }
} ;


template <class T>
class SimpSamp
{

    float *P;
    int cnt;

public:
    T *objs;


    SimpSamp()
    {
        P = (float*) malloc(sizeof(float)*1000);
        objs = (T*) malloc(sizeof(T)*1000);
    }
    ~SimpSamp()
    {
        free(P);
        free(objs);
    }

    inline void clear()
    {
        cnt = 1;
        P[0] = 0;
    }

    inline void add(float p, T obj)
    {
        P[cnt] = P[cnt-1] + p;
        objs[cnt-1] = obj;
        cnt++;
    }

    //   inline int draw()
    //   {
    //     float r = mtrand.frand()*P[cnt-1];
    //     for (int i = 1; i < cnt; i++)
    //     {
    //       if (r <= P[i])
    //         return i-1;
    //     }
    //     return cnt-2;
    //   }

    inline int draw()
    {
        float r = mtrand.frand()*P[cnt-1];
        int j;
        int rl = 1;
        int rh = cnt-1;
        while(rh != rl)
        {
            j = rl + (rh-rl)/2;
            if (r < P[j])
            {
                rh = j;
                continue;
            }
            if (r > P[j])
            {
                rl = j+1;
                continue;
            }
            break;
        }
        return rh-1;
    }





    inline T drawObj()
    {
        return objs[draw()];
    }

    inline bool isempty()
    {
        if (cnt == 1)
            return true;
        else
            return false;
    }


    float probFor(int idx)
    {
        return (P[idx+1]-P[idx])/P[cnt-1];
    }

    float probFor(T &t)
    {
        for (int i = 1; i< cnt;i++)
        {
            if (t == objs[i-1])
                return probFor(i-1);
        }
        return 0;
    }



};


class EndPoint
{
public:
    EndPoint()
    {}

    EndPoint(Particle *p,int ep)
    {
        this->p = p;
        this->ep = ep;
    }
    Particle *p;
    int ep;

    inline bool operator==(EndPoint P)
    {
        return (P.p == p) && (P.ep == ep);
    }
};

class Track
{
public:
    EndPoint track[1000];
    float energy;
    float proposal_probability;
    int length;

    void clear()
    {
        length = 0;
        energy = 0;
        proposal_probability = 1;
    }


    bool isequal(Track &t)
    {
        for (int i = 0; i < length;i++)
        {
            if (track[i].p != t.track[i].p || track[i].ep != t.track[i].ep)
                return false;
        }
        return true;
    }

};

float getMax(float *arr, int cnt)
{
    float max = arr[0];
    for (int i = 1; i < cnt; i++)
    {
        if (arr[i] > max)
            max = arr[i];
    }
    return max;
}



float getMin(float *arr, int cnt)
{
    float min = arr[0];
    for (int i = 1; i < cnt; i++)
    {
        if (arr[i] < min)
            min = arr[i];
    }
    return min;
}


int getArgMin(float *arr, int cnt)
{
    float min = arr[0];
    int idx = 0;
    for (int i = 1; i < cnt; i++)
    {
        if (arr[i] < min)
        {
            min = arr[i];
            idx = i;
        }
    }
    return idx;
}




inline float distLseg(pVector &R1,pVector &N1,pVector &R2,pVector &N2,float &len)
{

    pVector D = R1-R2;
    float beta  = N1*N2;
    float divisor = 1.001-beta*beta;
    float gamma1 = N1*D;
    float gamma2 = N2*D;
    float t,u;
    float EPdist[4];

    pVector Q;
    float dist = 102400000.0;

    while(true)
    {

        t = -(gamma1+beta*gamma2) / divisor;
        u =  (gamma1*beta+gamma2) / divisor;
        if (fabs(t) < len && fabs(u) < len)
        {
            Q = D +  N1*t - N2*u;
            dist = Q*Q;
            break;
        }

        beta = len*beta;

        t = beta - gamma1;
        if (fabs(t) < len)
        {
            Q = D +  N1*t - N2*len;
            float d = Q*Q;
            if (d < dist) dist = d;
        }

        t = -beta - gamma1;
        if (fabs(t) < len)
        {
            Q = D +  N1*t + N2*len;
            float d = Q*Q;
            if (d < dist) dist = d;
        }

        u = beta + gamma2;
        if (fabs(u) < len)
        {
            Q = D +  N1*len - N2*u;
            float d = Q*Q;
            if (d < dist) dist = d;
        }

        u = -beta + gamma2;
        if (fabs(u) < len)
        {
            Q = D -  N1*len - N2*u;
            float d = Q*Q;
            if (d < dist) dist = d;
        }

        if (dist != 102400000.0)
            break;


        EPdist[0] =  beta + gamma1 - gamma2;
        EPdist[1] = -beta + gamma1 + gamma2;
        EPdist[2] = -beta - gamma1 - gamma2;
        EPdist[3] =  beta - gamma1 + gamma2;
        int c = getArgMin(EPdist,4);
        if (c==0) {t = +len; u = +len; }
        if (c==1) {t = +len; u = -len; }
        if (c==2) {t = -len; u = +len; }
        if (c==3) {t = -len; u = -len; }
        Q = D +  N1*t - N2*u;
        dist = Q*Q;
        break;

    }


    return dist;

}


#endif


