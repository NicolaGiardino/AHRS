/******************************************************************/
//This file is part of The AHRS Project.
//
//Copyright © 2020 By Nicola di Gruttola Giardino. All rights reserved.
//@mail: nicoladgg@protonmail.com
//
//SOC-SPKF is free software: you can redistribute it and/or modify
//it under the terms of the GNU General Public License as published by
//the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.
//
//AHRS is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.
//
//You should have received a copy of the GNU General Public License
//along with The AHRS Project.  If not, see <https://www.gnu.org/licenses/>.
//
//In case of use of this project, I ask you to mention me, to whom it may concern.
/******************************************************************/

#include "matrix.h"

static float vec_mult(float*,float*,int);
static int row_scalar_multiply(Matrix *, int , float );

Matrix* Create(int r, int c) {
    if(r<0 || c<0){
        perror("Negative values");
        return -1;
    }
    
    Matrix *m = (Matrix*) malloc(sizeof(Matrix*));
    m->matrix=(float**)malloc(r*sizeof(float*));
    for (int i=0; i < r; i++) {
        m->matrix[i] =(float *)malloc(c * sizeof(float));
    }
    
    m->c = c;
    m->r = r;
    
    ZeroMat(m);
    return (Matrix*) m;
}

int Destroy(Matrix* m){
    if(m==NULL)
        return -1;
    Matrix* p = m;
    for (int i=0; i < p->r; i++) free(p->matrix[i]);
    free(p->matrix);
    
    return 0;
}

Matrix *inverse(Matrix *m1){
    Matrix *invert,*m;
    unsigned int i, j, l;
    double factor;
    if(m1 == NULL)
        return NULL;
    if((m1)->r != (m1)->c)
        return NULL;
    invert = identity((m1)->c);
    m=copy(m1);
    
    /* reduce each of the rows to get a lower triangle */
    for(i = 0; i < (m)->r; i++){
        for(j = i + 1; j < (m)->c; j++){
            if((m)->matrix[i][i] == 0){
                for(l=i+1; l < m->c; l++){
                    if(m->matrix[l][l] != 0){
                        rowSwap(m, i, l);
                        break;
                    }
                }
                continue;
            }
            factor = (m)->matrix[i][j]/((m)->matrix[i][i]);
            reduce(invert, i, j, factor);
            reduce((m), i, j, factor);
        }
    }
    /* now finish the upper triangle  */
    for(i = (m)->r - 1; i > 0; i--){
        for(j = i-1; j>=0; j--){
            if((m)->matrix[i][i] == 0)
                continue;
            if(j == -1)
                break;
            factor = (m)->matrix[i][j]/((m)->matrix[i][i]);
            reduce(invert, i, j, factor);
            reduce((m), i, j, factor);
        }
    }
    /* scale everything to 1 */
    for(i = 0; i < (m)->r; i++){
        if((m)->matrix[i][i]==0)
            continue;
        factor = 1/((m)->matrix[i][i]);
        row_scalar_multiply(invert, i, factor);
        row_scalar_multiply((m), i, factor);
    }
    return invert;
}

int ZeroMat(Matrix* m){
    if(m->r<0 || m->c<0)
        return -1;
    
    for (int i=0; i<m->r; i++) {
        for(int j=0;j<m->c;j++)
            m->matrix[i][j]=0;
    }
    
    return 0;
}

int sum(Matrix* m1, Matrix* m2){
    if(m1==NULL || m2==NULL)
        return -1;
    
    if(m1->r != m2->r || m1->c != m2->c)
        return -1;
    
    for (int i=0; i<m1->r; i++) {
        for (int j=0; j<m1->c; j++) {
            m1->matrix[i][j]+=m2->matrix[i][j];
        }
    }
    
    return 0;
}

int subtract(Matrix* m1, Matrix* m2){
    if(m1==NULL || m2==NULL)
        return -1;
    
    if(m1->r != m2->r || m1->c != m2->c)
        return -1;
    
    for (int i=0; i<m1->r; i++) {
        for (int j=0; j<m1->c; j++) {
            m1->matrix[i][j]-=m2->matrix[i][j];
        }
    }
    
    return 0;
}

int sc_multiply(Matrix* m1, float f){
    if(m1==NULL)
        return -1;
    
    for (int i=0; i<m1->r; i++) {
        for (int j=0; j<m1->c; j++) {
            m1->matrix[i][j]*=f;
        }
    }
    
    return 0;
}

int equals(Matrix* m1, Matrix* m2){
    if(m1==NULL || m2==NULL)
        return -1;
    
    if(m1->r != m2->r || m1->c != m2->c)
        return -1;
    
    for (int i=0; i<m1->r; i++) {
        for (int j=0; j<m1->c; j++) {
            if(m1->matrix[i][j]!=m2->matrix[i][j])
                return 0;
        }
    }
    
    return 1;
}

Matrix *multiply(Matrix *m1, Matrix *m2){
    Matrix *product;
    unsigned int i, j;
    if(m1 == NULL || m2 == NULL)
        return -1;
    if(m1->c != m2->r)
        return -1;
    product = Create(m1->r, m2->c);
    for (int i = 0; i < m1->r; ++i) {
        for (int j = 0; j < m2->c; ++j) {
            for (int k = 0; k < m1->c; ++k) {
                product->matrix[i][j] += m1->matrix[i][k] * m2->matrix[k][j];
            }
        }
    }
    return product;
}

Matrix* transpose(Matrix* m){
    if(m==NULL)
        return -1;
    
    Matrix* t;
    t=Create(m->c,m->r);
    
    for (int i=0; i<t->r; i++) {
        for (int j=0; j<t->c; j++) {
            t->matrix[i][j]=m->matrix[j][i];
        }
    }
    
    return t;
}

Matrix* identity(int lenght){
    if(lenght<=0)
        return -1;
    
    Matrix* m;
    m=Create(lenght,lenght);
    
    for (int i=0; i<lenght; i++) {
        for (int j=0; j<lenght; j++) {
            m->matrix[i][j]=(i==j);
        }
    }
    
    return m;
}

float determinant(Matrix* m){
    if(m==NULL)
        return -1;
    
    Matrix* L;
    L=Create(m->r,m->c);
    Matrix* U;
    U=Create(m->r,m->c);
    
    LU(m,L,U);
    
    float detL=0;
    float detU=0;
    
    for (int i=0; i<m->r; i++) {
        float sum1=0;
        float sum2=0;
        for (int j=0; j<m->r-i; j++) {
            sum1*=L->matrix[i][j];
            sum2*=U->matrix[i][j];
        }
        detL+=sum1;
        detU+=sum2;
    }
    
    return detL*detU;
}

int LU(Matrix* m, Matrix* L, Matrix* U){
    
    ZeroMat(L);
    ZeroMat(U);
    
    for (int i=0; i<m->c; i++) {
        int sum=0;
        
        for(int k=i;k<m->r;k++){
            
            //sum of Lij*Ujk
            for (int j=0; j<i; j++)
                sum+=(L->matrix[i][j]*U->matrix[j][k]);
            
            U->matrix[i][k]=m->matrix[i][k]-sum;
        }
        for(int k=i;k<m->r;k++){
            if(i==k)
                L->matrix[i][i]=1;
            
            else{
                int sum=0;
                for (int j=0;j<i;j++)
                    sum+=(L->matrix[k][i]*U->matrix[j][i]);
                
                L->matrix[k][i]=(m->matrix[k][i]-sum/U->matrix[i][i]);
            }
        }
    }
    return 0;
}

float *eigenvalues(Matrix *m){
    float *values, factor;
    Matrix *r;
    unsigned int i, j, l;
    if(m == NULL)
        return -1;
    if(m->r != m->c)
        return -1;
    
    values = malloc(sizeof(double)*m->c);
    r = copy(m);
    /* reduce each of the rows to get a lower triangle */
    for(i = 0; i < r->r; i++){
        for(j = i + 1; j < r->c; j++){
            if(r->matrix[i][i] == 0){
                for(l = i+1; l < r->c; l++){
                    if(r->matrix[l][l] != 0){
                        rowSwap(r, i, l);
                        break;
                    }
                }
                continue;
            }
            factor = r->matrix[i][j]/(r->matrix[i][i]);
            reduce(r, i, j, factor);
        }
    }
    for(i = 0; i < r->r; i++)
        values[i] = r->matrix[i][i];
    
    return values;
}

int print(Matrix* m){
    if(m==NULL)
        return -1;
    
    for (int i=0; i<m->r; i++) {
        for (int j=0; j<m->c; j++) {
            printf("%f\t",m->matrix[i][j]);
        }
        printf("\n");
    }
    
    return 0;
}

static float vec_mult(float* v1,float* v2 ,int lenght){
    if(v1==NULL || v2==NULL || lenght<=0)
        return -1;
    
    float v=0;
    
    for (int i=0; i<lenght; i++) {
        v+=(v1[i]*v2[i]);
    }
    
    return v;
}

Matrix* copy(Matrix* m){
    if(m==NULL)
        return -1;
    
    Matrix* c;
    c=Create(m->r,m->c);
    
    for (int i=0; i<m->r; i++) {
        for (int j=0; j<m->c; j++)
            c->matrix[i][j]=m->matrix[i][j];
    }
    
    return c;
}

int rowSwap(Matrix* m,int a,int b){
    float temp;
    unsigned int i;
    
    if(m == NULL)
        return -1;
    
    if(m->c <= a || m->c <= b)
        return -1;
    
    for(i = 0; i < m->r; i++){
        temp = m->matrix[i][a];
        m->matrix[i][a] = m->matrix[i][b];
        m->matrix[i][b] = temp;
    }
    return 0;
}

int reduce(Matrix* m,int a,int b,float f){
    int i;
    if(m == NULL)
        return -1;
    
    if(m->c < a || m->c < b)
        return -1;
    for(i = 0; i < m->r; i++){
        m->matrix[i][b]  -= m->matrix[i][a]*f;
    }
    
    return 0;
}

Matrix* Chol(Matrix* m){
    Matrix *L;
    L = Create(m->r,m->c);
    
    if (L == NULL)
        return -1;
    
    for (int i = 0; i < L->c; i++)
        for (int j = 0; j < (i+1); j++) {
            double s = 0;
            for (int k = 0; k < j; k++)
                s += L->matrix[i][k] * L->matrix[j][k];
            L->matrix[i][j] = (i == j) ?
            sqrt((m->matrix[i][i]) - s) :
            (1.0 / L->matrix[j][j] * (m->matrix[i][j] - s));
        }
    return L;
}

Matrix* sqrtm(Matrix* m){
    if(m==NULL)
        return -1;
    
    Matrix* a;
    a=Create(m->r,m->c);
    
    for (int i=0; i<m->r; i++) {
        for (int j=0; j<m->c; j++)
            a->matrix[i][j]=sqrt(m->matrix[i][j]);
    }
    
    return a;
}

Matrix* msum(Matrix* m1, Matrix* m2){
    if(m1==NULL || m2==NULL)
        return -1;
    
    if(m1->r != m2->r || m1->c != m2->c)
        return -1;
    
    Matrix* s=malloc(sizeof(Matrix));
    s=Create(m1->r,m1->c);
    
    for (int i=0; i<m1->r; i++) {
        for (int j=0; j<m1->c; j++) {
            s->matrix[i][j]=m1->matrix[i][j]+m2->matrix[i][j];
        }
    }
    
    return s;
}

Matrix* m_sc_multiply(Matrix* m1, float f){
    if(m1==NULL)
        return -1;
    
    Matrix* s;
    s=Create(m1->r,m1->c);
    
    for (int i=0; i<m1->r; i++) {
        for (int j=0; j<m1->c; j++) {
            s->matrix[i][j]=m1->matrix[i][j]*f;
        }
    }
    
    return s;
}

Matrix* expm(Matrix* m,float e){
    if(m==NULL)
        return -1;
    
    Matrix* a;
    a=Create(m->r,m->c);
    
    for (int i=0; i<m->r; i++) {
        for (int j=0; j<m->c; j++)
            a->matrix[i][j]=pow(m->matrix[i][j],e);
    }
    
    return a;
}

Matrix* msubtract(Matrix* m1,Matrix* m2){
    if(m1==NULL || m2==NULL)
        return -1;
    
    if(m1->r != m2->r || m1->c != m2->c)
        return -1;
    
    Matrix* s=malloc(sizeof(Matrix));
    s=Create(m1->r,m1->c);
    
    for (int i=0; i<m1->r; i++) {
        for (int j=0; j<m1->c; j++) {
            s->matrix[i][j]=m1->matrix[i][j]-m2->matrix[i][j];
        }
    }
    
    return s;
}

static int row_scalar_multiply(Matrix *m, int row, float factor){
    int i;
    if(m == NULL)
        return -1;
    if(m->c <= row)
        return -1;
    for(i = 0; i < m->r; i++)
        m->matrix[i][row] *= factor;
    return 0;
}

Matrix* blkdiag(Matrix* m1,Matrix* m2,Matrix* m3){
    if(m1==NULL || m2==NULL || m3==NULL)
        return -1;
    
    if(m1->r<0 || m1->c<0 || m2->r<0 || m2->c<0 || m3->r<0 || m3->c<0)
        return -1;
    
    Matrix* m = Create(m1->r + m2->r + m3->r, m1->c + m2->c + m3->c);
    
    for (int i=0; i<m->r; i++) {
        for (int j=0; j<m->c; j++) {
            if(j<m1->c && i<m1->r)
                m->matrix[i][j]=m1->matrix[i][j];
            else if(j<(m1->c+m2->c) && i<(m1->r+m2->r) && j>=m1->c && i>=m1->r)
                m->matrix[i][j]=m2->matrix[i-(m1->r)][j-(m1->c)];
            else if(j<(m1->c+m2->c+m3->c) && i<(m1->r+m2->r+m3->r) && j>=(m1->c+m2->c) && i>=(m1->r+m2->r))
                m->matrix[i][j]=m3->matrix[i-(m1->r+m2->r)][j-(m1->c+m2->c)];
        }
    }
    
    return m;
}

Matrix* diag(Matrix* m){
    if(m==NULL)
        return -1;
    
    if(m->r<0 || m->c!=1)
        return -1;
    
    Matrix* d=Create(m->r,m->r);
    
    for (int i=0; i<m->r; i++) {
        d->matrix[i][i]=m->matrix[0][i];
    }
    
    return d;
}
