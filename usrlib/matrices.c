/***********************************************************************************
* This file is part of The AHRS Project.                                           *
*                                                                                  *
* Copyright © 2020 By Nicola di Gruttola Giardino. All rights reserved.            *
* @mail: nicoladgg@protonmail.com                                                  *
*                                                                                  *
* AHRS is free software: you can redistribute it and/or modify                     *
* it under the terms of the GNU General Public License as published by             *
* the Free Software Foundation, either version 3 of the License, or                *
* (at your option) any later version.                                              *
*                                                                                  *
* AHRS is distributed in the hope that it will be useful,                          *
* but WITHOUT ANY WARRANTY; without even the implied warranty of                   *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                    *
* GNU General Public License for more details.                                     *
*                                                                                  *
* You should have received a copy of the GNU General Public License                *
* along with The AHRS Project.  If not, see <https://www.gnu.org/licenses/>.       *
*                                                                                  *
* In case of use of this project, I ask you to mention me, to whom it may concern. *
***********************************************************************************/

/****************************************************************************************************
* FILE NAME: matrices.c                                                                             *
*                                                                                                   *
* PURPOSE: This library gives all the tools to implement a Matrix Object                            *
*                                                                                                   *
* FILE REFERENCES:                                                                                  *
*                                                                                                   *
*   Name    I/O     Description                                                                     *
*   ----    ---     -----------                                                                     *
*   none                                                                                            *
*                                                                                                   *
*                                                                                                   *
* EXTERNAL VARIABLES:                                                                               *
*                                                                                                   *
* Source: <matrices.h>                                                                              *
*                                                                                                   *
* Name          Type    IO Description                                                              *
* ------------- ------- -- -----------------------------                                            *
*   m               Matrix      Matrix object, contains                                             *
*                                a float**, being the matrix, and 2 int, being rows and columns     *
*                                                                                                   *
*                                                                                                   *
* STATIC VARIABLES:                                                                                 *
*                                                                                                   *
*   Name     Type       I/O      Description                                                        *
*   ----     ----       ---      -----------                                                        *
*   none                                                                                            *
*                                                                                                   *
* EXTERNAL REFERENCES:                                                                              *
*                                                                                                   *
*  Name                       Description                                                           *
*  -------------              -----------                                                           *
*  none                                                                                             *
*                                                                                                   *
* ABNORMAL TERMINATION CONDITIONS, ERROR AND WARNING MESSAGES:                                      *
*    none, compliant with the standard ISO9899:1999                                                 *
*                                                                                                   *
* ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS: tbd                                                       *
*                                                                                                   *
* NOTES: see documentations                                                                         *
*                                                                                                   *
* REQUIREMENTS/FUNCTIONAL SPECIFICATIONS REFERENCES:                                                *
*                                                                                                   *
* DEVELOPMENT HISTORY:                                                                              *
*                                                                                                   *
*   Date          Author            Change Id     Release     Description Of Change                 *
*   ----          ------            ---------     ------      ----------------------                *
*   14-11-2019    N.di Gruttola      1               1         Initial commit                       *
*                   Giardino                                                                        *
*   02-03-2020    N. di Gruttola     2               1.1       Modified libraries, added            *
*                   Giardino                                    my own matrix library               *
*   04-07-2020    N.di Gruttola                      1.2       Added comments, code satisfies       *
*                  Giardino                                     iso9899:1999, as requested per      *
*                                                               MISRA-C:2004                        *
*                                                                                                   *
*                                                                                                   *
*                                                                                                   *
****************************************************************************************************/

/* Include Global Parameters */

#include "matrix.h"

/* Declare Prototypes */

static float vec_mult             (float *, float *, int);
static int   row_scalar_multiply  (Matrix *, int , float);

/********************************************************************************
*                                                                               *
* FUNCTION NAME: Create                                                         *
*                                                                               *
* PURPOSE: Creates the object Matrix, and then fills it with zeros              *
*           returning the pointer to the created matrix                         *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* r         int          I      Number of rows                                  *
* c         int          I      Number of columns                               *
*                                                                               *
* RETURN VALUE: Matrix*                                                         *
*                                                                               *
********************************************************************************/

Matrix* Create(int r, int c)
{
    if(r<0 || c<0)
    {
        perror("Negative values");
        return NULL;
    }

    Matrix *m = (Matrix*) malloc(sizeof(Matrix*));
    m->matrix=(float**)malloc(r*sizeof(float*));
    for (int i=0; i < r; i++) 
    {
        m->matrix[i] =(float *)malloc(c * sizeof(float));
    }

    m->c = c;
    m->r = r;
    ZeroMat(m);
    return (Matrix*) m;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: Destroy                                                        *
*                                                                               *
* PURPOSE: Destroys the Object                                                  *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m         Matrix*      I      Matrix to free                                  *
*                                                                               *
* RETURN VALUE: int                                                             *
*                true or false                                                  *
*                                                                               *
********************************************************************************/

int Destroy(Matrix* m)
{
    if(m==NULL)
        return -1;
    Matrix* p = m;
    for (int i=0; i < p->r; i++) free(p->matrix[i]);
    free(p->matrix);

    return 0;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: inverse                                                        *
*                                                                               *
* PURPOSE: Creates the inverse of the matrix given as input,                    *
*            returning the pointer to the inverted matrix                       *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m         Matrix*      I      Pointer to the object to invert                 *
*                                                                               *
* RETURN VALUE: Matrix*                                                         *
*                                                                               *
********************************************************************************/

Matrix *inverse(Matrix *m1)
{
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
    for(i = 0; i < (m)->r; i++)
    {
        for(j = i + 1; j < (m)->c; j++)
        {
            if((m)->matrix[i][i] == 0)
            {
                for(l=i+1; l < m->c; l++)
                {
                    if(m->matrix[l][l] != 0)
                    {
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
    for(i = (m)->r - 1; i > 0; i--)
    {
        for(j = i-1; j>=0; j--)
        {
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
    for(i = 0; i < (m)->r; i++)
    {
        if((m)->matrix[i][i]==0)
            continue;
        factor = 1/((m)->matrix[i][i]);
        row_scalar_multiply(invert, i, factor);
        row_scalar_multiply((m), i, factor);
    }
    return invert;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: ZeroMat                                                        *
*                                                                               *
* PURPOSE: Fills the given matrix with zeros                                    *
*            returns -1 if failed, 0 if successfull                             *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m         Matrix*      IO     Pointer to the object to fill                   *
*                                                                               *
* RETURN VALUE: int                                                             *
*                                                                               *
********************************************************************************/

int ZeroMat(Matrix* m)
{
    if(m->r<0 || m->c<0)
        return -1;

    for (int i=0; i<m->r; i++)
    {
        for(int j=0;j<m->c;j++)
            m->matrix[i][j]=0;
    }

    return 0;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: sum                                                            *
*                                                                               *
* PURPOSE: Sums the second matrix to the first matrix                           *
*            returns -1 if failed, 0 if successfull                             *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m1        Matrix*      IO     Pointer to the 1st object to sum                *
* m2        Matrix*      I      Pointer to the 2nd object to sum                *
*                                                                               *
* RETURN VALUE: int                                                             *
*                                                                               *
********************************************************************************/

int sum(Matrix* m1, Matrix* m2)
{
    if(m1==NULL || m2==NULL)
        return -1;

    if(m1->r != m2->r || m1->c != m2->c)
        return -1;

    for (int i=0; i<m1->r; i++)
    {
        for (int j=0; j<m1->c; j++) 
            m1->matrix[i][j]+=m2->matrix[i][j];
    }

    return 0;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: subtract                                                       *
*                                                                               *
* PURPOSE: Subtracts the second matrix to the first matrix                      *
*            returns -1 if failed, 0 if successfull                             *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m1        Matrix*      IO     Pointer to the 1st object to subtract           *
* m2        Matrix*      I      Pointer to the 2nd object to subtract           *
*                                                                               *
* RETURN VALUE: int                                                             *
*                                                                               *
********************************************************************************/

int subtract(Matrix* m1, Matrix* m2)
{
    if(m1==NULL || m2==NULL)
        return -1;

    if(m1->r != m2->r || m1->c != m2->c)
        return -1;

    for (int i=0; i<m1->r; i++) 
    {
        for (int j=0; j<m1->c; j++)
            m1->matrix[i][j]-=m2->matrix[i][j];
    }

    return 0;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: sc_multiply                                                    *
*                                                                               *
* PURPOSE: Multiplies the scalar to the Matrix                                  *
*            returns -1 if failed, 0 if successfull                             *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m1        Matrix*      IO     Pointer to the object to multiply               *
* f         float        I      Scalar to multiply                              *
*                                                                               *
* RETURN VALUE: int                                                             *
*                                                                               *
********************************************************************************/

int sc_multiply(Matrix* m1, float f)
{
    if(m1==NULL)
        return -1;

    for (int i=0; i<m1->r; i++) 
    {
        for (int j=0; j<m1->c; j++)
            m1->matrix[i][j]*=f;
    }

    return 0;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: equals                                                         *
*                                                                               *
* PURPOSE: Compares the 2 matrices                                              *
*            returns -1 if failed, 0 if successfull                             *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m1        Matrix*      I      Pointer to the 1st object                       *
* m2        Matrix*      I      Pointer to the 2nd object                       *
*                                                                               *
* RETURN VALUE: int                                                             *
*                                                                               *
********************************************************************************/

int equals(Matrix* m1, Matrix* m2)
{
    if(m1==NULL || m2==NULL)
        return -1;

    if(m1->r != m2->r || m1->c != m2->c)
        return -1;

    for (int i=0; i<m1->r; i++) 
    {
        for (int j=0; j<m1->c; j++)
        {
            if(m1->matrix[i][j]!=m2->matrix[i][j])
                return 0;
        }
    }

    return 1;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: multiply                                                       *
*                                                                               *
* PURPOSE: Multiplies the 2 matrices                                            *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m1        Matrix*      I      Pointer to the 1st object to multiply           *
* m2        Matrix*      I      Pointer to the 2nd object to multiply           *
*                                                                               *
* RETURN VALUE: Matrix*                                                         *
*                                                                               *
********************************************************************************/
Matrix *multiply(Matrix *m1, Matrix *m2)
{
    Matrix *product;
    unsigned int i, j;
    if(m1 == NULL || m2 == NULL)
        return NULL;
    if(m1->c != m2->r)
        return NULL;
    product = Create(m1->r, m2->c);
    for (int i = 0; i < m1->r; ++i)
    {
        for (int j = 0; j < m2->c; ++j) 
        {
            for (int k = 0; k < m1->c; ++k)
                product->matrix[i][j] += m1->matrix[i][k] * m2->matrix[k][j];
        }
    }
    return product;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: transpose                                                      *
*                                                                               *
* PURPOSE: Transposes the matrix                                                *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m         Matrix*      I      Pointer to the object                           *
*                                                                               *
* RETURN VALUE: Matrix*                                                         *
*                                                                               *
********************************************************************************/
Matrix* transpose(Matrix* m)
{
    if(m==NULL)
        return NULL;

    Matrix* t;
    t=Create(m->c,m->r);

    for (int i=0; i<t->r; i++)
    {
        for (int j=0; j<t->c; j++)
            t->matrix[i][j]=m->matrix[j][i];
    }

    return t;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: identity                                                       *
*                                                                               *
* PURPOSE: Returns the Identity matrix                                          *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* lenght    int          I      lenght of I                                     *
*                                                                               *
* RETURN VALUE: Matrix*                                                         *
*                                                                               *
********************************************************************************/
Matrix* identity(int lenght)
{
    if(lenght<=0)
        return NULL;

    Matrix* m;
    m=Create(lenght,lenght);

    for (int i=0; i<lenght; i++) 
    {
        for (int j=0; j<lenght; j++)
            m->matrix[i][j]=(i==j);
    }

    return m;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: determinant                                                    *
*                                                                               *
* PURPOSE: Computes the determinant using LU decomposition                      *
*            returns -1 if failed                                               *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m         Matrix*      I      Pointer to the object                           *
*                                                                               *
* RETURN VALUE: float                                                           *
*                                                                               *
********************************************************************************/

float determinant(Matrix* m)
{
    if(m==NULL)
        return -1;

    Matrix* L;
    L=Create(m->r,m->c);
    Matrix* U;
    U=Create(m->r,m->c);

    LU(m,L,U);

    float detL=0;
    float detU=0;

    for (int i=0; i<m->r; i++) 
    {
        float sum1=0;
        float sum2=0;
        for (int j=0; j<m->r-i; j++)
        {
            sum1*=L->matrix[i][j];
            sum2*=U->matrix[i][j];
        }
        detL+=sum1;
        detU+=sum2;
    }

    return detL*detU;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: LU                                                             *
*                                                                               *
* PURPOSE: Compares the LU transformation                                       *
*            returns -1 if failed, 0 if successfull                             *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m         Matrix*      I      Pointer to the object                           *
* L         Matrix*      O      Pointer to the L decomposed object              *
* U         Matrix*      O      Pointer to the U decomposed object              *
*                                                                               *
* RETURN VALUE: int                                                             *
*                                                                               *
********************************************************************************/
int LU(Matrix* m, Matrix* L, Matrix* U)
{

    ZeroMat(L);
    ZeroMat(U);

    for (int i=0; i<m->c; i++) 
    {
        int sum=0;

        for(int k=i;k<m->r;k++)
        {

            //sum of Lij*Ujk
            for (int j=0; j<i; j++)
                sum+=(L->matrix[i][j]*U->matrix[j][k]);

            U->matrix[i][k]=m->matrix[i][k]-sum;
        }
        for(int k=i;k<m->r;k++)
        {
            if(i==k)
                L->matrix[i][i]=1;

            else
            {
                int sum=0;
                for (int j=0;j<i;j++)
                    sum+=(L->matrix[k][i]*U->matrix[j][i]);

                L->matrix[k][i]=(m->matrix[k][i]-sum/U->matrix[i][i]);
            }
        }
    }
    return 0;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: eigenvalues                                                    *
*                                                                               *
* PURPOSE: Computes the eigenvalues of the matrix                               *
*           returning -1 if failed,                                             *
*           the pointer to the eigenvalues if successfull                       *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m         Matrix*      I      Pointer to the 1st object                       *
*                                                                               *
* RETURN VALUE: float*                                                          *
*                                                                               *
********************************************************************************/
float *eigenvalues(Matrix *m)
{
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
    for(i = 0; i < r->r; i++)
    {
        for(j = i + 1; j < r->c; j++)
        {
            if(r->matrix[i][i] == 0)
            {
                for(l = i+1; l < r->c; l++)
                {
                    if(r->matrix[l][l] != 0)
                    {
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

/********************************************************************************
*                                                                               *
* FUNCTION NAME: equals                                                         *
*                                                                               *
* PURPOSE: Prints the matrix                                                    *
*            returns -1 if failed, 0 if successfull                             *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m         Matrix*      I      Pointer to the object                           *
*                                                                               *
* RETURN VALUE: int                                                             *
*                                                                               *
********************************************************************************/
int print(Matrix* m)
{
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

/********************************************************************************
*                                                                               *
* FUNCTION NAME: vec_mult                                                       *
*                                                                               *
* PURPOSE: Multiplies two vectors, declared as static,                          *
*           to be used in this header only                                      *
*           returning -1 if failed,                                             *
*           the pointer to the scalar if successfull                            *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* v1        float*       I      Pointer to the 1st object                       *
* v2        float*       I      Pointer to the 2nd object                       *
* lenght    int          I      Lenght of the vectors
*                                                                               *
* RETURN VALUE: float                                                           *
*                                                                               *
********************************************************************************/
static float vec_mult(float* v1,float* v2 ,int lenght)
{
    if(v1==NULL || v2==NULL || lenght<=0)
        return -1;

    float v=0;

    for (int i=0; i<lenght; i++) 
        v+=(v1[i]*v2[i]);

    return v;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: copy                                                           *
*                                                                               *
* PURPOSE: Copies the matrix into another                                       *
*           returning NULL if failed,                                           *
*           the pointer to the object if successfull                            *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m         Matrix*      I      Pointer to the object                           *
*                                                                               *
* RETURN VALUE: int                                                             *
*                                                                               *
********************************************************************************/
Matrix* copy(Matrix* m)
{
    if(m==NULL)
        return NULL;

    Matrix* c;
    c=Create(m->r,m->c);

    for (int i=0; i<m->r; i++)
    {
        for (int j=0; j<m->c; j++)
            c->matrix[i][j]=m->matrix[i][j];
    }

    return c;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: rowSwap                                                        *
*                                                                               *
* PURPOSE: Swaps two rows of the matrix                                         *
*           returning -1 if failed, 0 if successfull                            *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m         Matrix*      IO     Pointer to the object                           *
* a         int          I      1st row to swap                                 *
* b         int          I      2nd row to swap                                 *
*                                                                               *
* RETURN VALUE: int                                                             *
*                                                                               *
********************************************************************************/
int rowSwap(Matrix* m,int a,int b)
{
    float temp;
    unsigned int i;

    if(m == NULL)
        return -1;

    if(m->c <= a || m->c <= b)
        return -1;

    for(i = 0; i < m->r; i++)
    {
        temp = m->matrix[i][a];
        m->matrix[i][a] = m->matrix[i][b];
        m->matrix[i][b] = temp;
    }
    return 0;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: reduce                                                         *
*                                                                               *
* PURPOSE: reduces a piece of the matrix by a factor f                          *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m         Matrix*      I      Pointer to the object                           *
* a         int          I      First column                                    *
* b         int          I      Second column                                   *
*                                                                               *
* RETURN VALUE: int                                                             *
*                                                                               *
********************************************************************************/
int reduce(Matrix* m,int a,int b,float f)
{
    int i;
    if(m == NULL)
        return -1;

    if(m->c < a || m->c < b)
        return -1;

    for(i = 0; i < m->r; i++)
        m->matrix[i][b]  -= m->matrix[i][a]*f;

    return 0;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: Chol                                                           *
*                                                                               *
* PURPOSE: Computes the Cholesky factorization of the matrix                    *
*           returning -1 if failed,                                             *
*           the pointer to the object if successfull                            *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m         Matrix*      I      Pointer to the object                           *
*                                                                               *
* RETURN VALUE: Matrix*                                                         *
*                                                                               *
********************************************************************************/
Matrix* Chol(Matrix* m)
{
    Matrix *L;
    L = Create(m->r,m->c);

    if (L == NULL)
        return NULL;

    for (int i = 0; i < L->c; i++)
    {
        for (int j = 0; j < (i + 1); j++) 
        {
            float s = 0;
            for (int k = 0; k < j; k++)
                s += L->matrix[i][k] * L->matrix[j][k];
            L->matrix[i][j] = (i == j) ?
                sqrt((m->matrix[i][i]) - s) :
                (1.0 / L->matrix[j][j] * (m->matrix[i][j] - s));
        }
    }

    return L;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: sqrtm                                                          *
*                                                                               *
* PURPOSE: Computes the square root of the matrix's elements                    *
*           returning -1 if failed,                                             *
*           the pointer to the object if successfull                            *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m         Matrix*      I      Pointer to the object                           *
*                                                                               *
* RETURN VALUE: Matrix*                                                         *
*                                                                               *
********************************************************************************/
Matrix* sqrtm(Matrix* m)
{
    if(m==NULL)
        return NULL;

    Matrix* a;
    a=Create(m->r,m->c);

    for (int i=0; i<m->r; i++)
    {
        for (int j=0; j<m->c; j++)
            a->matrix[i][j]=sqrt(m->matrix[i][j]);
    }

    return a;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: msum                                                           *
*                                                                               *
* PURPOSE: Computes the sum of the matrices                                     *
*           returning NULL if failed,                                           *
*           the pointer to the object if successfull                            *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m1        Matrix*      I      Pointer to the 1st object                       *
* m2        Matrix*      I      Pointer to the 2nd object                       *
*                                                                               *
* RETURN VALUE: Matrix*                                                         *
*                                                                               *
********************************************************************************/
Matrix* msum(Matrix* m1, Matrix* m2)
{
    if(m1==NULL || m2==NULL)
        return NULL;

    if(m1->r != m2->r || m1->c != m2->c)
        return NULL;

    Matrix* s=malloc(sizeof(Matrix));
    s=Create(m1->r,m1->c);

    for (int i=0; i<m1->r; i++) 
    {
        for (int j=0; j<m1->c; j++)
            s->matrix[i][j]=m1->matrix[i][j]+m2->matrix[i][j];
    }

    return s;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: m_sc_multiply                                                  *
*                                                                               *
* PURPOSE: Multiplies the matrix to a scalar                                    *
*           returning NULL if failed,                                           *
*           the pointer to the object if successfull                            *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m1        Matrix*      I      Pointer to the 1st object                       *
* f         float        I      Multiplier                                      *
*                                                                               *
* RETURN VALUE: Matrix*                                                         *
*                                                                               *
********************************************************************************/
Matrix* m_sc_multiply(Matrix* m1, float f)
{
    if(m1==NULL)
        return NULL;

    Matrix* s;
    s=Create(m1->r,m1->c);

    for (int i=0; i<m1->r; i++) 
    {
        for (int j=0; j<m1->c; j++) 
            s->matrix[i][j]=m1->matrix[i][j]*f;
    }

    return s;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: expm                                                           *
*                                                                               *
* PURPOSE: Computes the exponential of the matrix's elements                    *
*           returning NULL if failed,                                           *
*           the pointer to the object if successfull                            *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m1        Matrix*      I      Pointer to the 1st object                       *
* f         float        I      Exponential                                     *
*                                                                               *
* RETURN VALUE: Matrix*                                                         *
*                                                                               *
********************************************************************************/
Matrix* expm(Matrix* m,float e)
{
    if(m==NULL)
        return NULL;

    Matrix* a;
    a=Create(m->r,m->c);

    for (int i=0; i<m->r; i++) 
    {
        for (int j=0; j<m->c; j++)
            a->matrix[i][j]=pow(m->matrix[i][j],e);
    }

    return a;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: msubtract                                                      *
*                                                                               *
* PURPOSE: Computes the subtraction of the matrices                             *
*           returning NULL if failed,                                           *
*           the pointer to the object if successfull                            *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m1        Matrix*      I      Pointer to the 1st object                       *
* m2        Matrix*      I      Pointer to the 2nd object                       *
*                                                                               *
* RETURN VALUE: Matrix*                                                         *
*                                                                               *
********************************************************************************/
Matrix* msubtract(Matrix* m1,Matrix* m2)
{
    if(m1==NULL || m2==NULL)
        return NULL;

    if(m1->r != m2->r || m1->c != m2->c)
        return NULL;

    Matrix* s=malloc(sizeof(Matrix));
    s=Create(m1->r,m1->c);

    for (int i=0; i<m1->r; i++)
    {
        for (int j=0; j<m1->c; j++) 
            s->matrix[i][j]=m1->matrix[i][j]-m2->matrix[i][j];
    }

    return s;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: row_scalar_multiply                                            *
*                                                                               *
* PURPOSE: multiplies a row by a factor                                         *
*            returns -1 if failed, 0 if successfull                             *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m         Matrix*      I      Pointer to the 1st object                       *
* row       float        I      No. of row                                      *
* factor    float        I      factor to multiply                              *
*                                                                               *
* RETURN VALUE: int                                                             *
*                                                                               *
********************************************************************************/
static int row_scalar_multiply(Matrix *m, int row, float factor)
{
    int i;
    if(m == NULL)
        return -1;
    if(m->c <= row)
        return -1;
    for(i = 0; i < m->r; i++)
        m->matrix[i][row] *= factor;
    return 0;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: blkdiag                                                        *
*                                                                               *
* PURPOSE: Computes the block diagonal of the matrices                          *
*           returning NULL if failed,                                           *
*           the pointer to the object if successfull                            *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m1        Matrix*      I      Pointer to the 1st object                       *
* m2        Matrix*      I      Pointer to the 2nd object                       *
* m2        Matrix*      I      Pointer to the 3rd object                       *
*                                                                               *
* RETURN VALUE: Matrix*                                                         *
*                                                                               *
********************************************************************************/
Matrix* blkdiag(Matrix* m1,Matrix* m2,Matrix* m3)
{
    if(m1==NULL || m2==NULL || m3==NULL)
        return NULL;

    if(m1->r<0 || m1->c<0 || m2->r<0 || m2->c<0 || m3->r<0 || m3->c<0)
        return NULL;

    Matrix* m = Create(m1->r + m2->r + m3->r, m1->c + m2->c + m3->c);

    for (int i=0; i<m->r; i++)
    {
        for (int j=0; j<m->c; j++) 
        {
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

/********************************************************************************
*                                                                               *
* FUNCTION NAME: diag                                                           *
*                                                                               *
* PURPOSE: Computes the diagonal of the matrix                                  *
*           returning NULL if failed,                                           *
*           the pointer to the object if successfull                            *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* m1        Matrix*      I      Pointer to the 1st object                       *
*                                                                               *
* RETURN VALUE: Matrix*                                                         *
*                                                                               *
********************************************************************************/
Matrix* diag(Matrix* m)
{
    if(m==NULL)
        return NULL;

    if(m->r<0 || m->c!=1)
        return NULL;

    Matrix* d=Create(m->r,m->r);

    for (int i=0; i<m->r; i++)
        d->matrix[i][i]=m->matrix[i][0];

    return d;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: interp1                                                        *
*                                                                               *
* PURPOSE: Computes the interpolation of a set of points                        *
*           returning -1 if failed,                                             *
*           the point if successfull                                            *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* x_new     float        I      Point to interpolate                            *
* x         float*       I      Vector of x points                              *
* y         float*       I      Vector of y points                              *
*                                                                               *
* RETURN VALUE: float                                                           *
*                                                                               *
********************************************************************************/
float interp1(float x_new, float* x, float* y)
{

    int i;

    for (i = 0; i < sizeof(x) && x_new <= x[i]; i++);

    return (y[i] + ((x_new - x[i]) * (y[i + 1] - y[i]) / (x[i + 1] - x[i])));

}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: CsvToFloat                                                     *
*                                                                               *
* PURPOSE: Reads from a CSV file, then passes the floats into a file            *
*           returning NULL if failed,                                           *
*           the pointer to the values if successfull                            *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* path      const char*  I      Path of the csv file                            *
*                                                                               *
* RETURN VALUE: float*                                                          *
*                                                                               *
********************************************************************************/
float* CsvToFloat(const char* path)
{
    FILE* myFile;
    int n = 0, i = 0;
    float* val;
    float values;

    myFile = fopen(path, "r");
    if (myFile == NULL)
    {
        perror("failed to open file");
        return NULL;
    }

    while (fscanf(myFile, "%f", &values) == 1) 
    {
        n++;
        fscanf(myFile, ",");
    }

    val = malloc(n + 1);

    while (fscanf(myFile, "%f", &val[i++]) == 1)
    {
        fscanf(myFile, ",");
    }

    fclose(myFile);

    return val;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: equals                                                         *
*                                                                               *
* PURPOSE: Sets the seed for a random values generator                          *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* s         const float  I      Seed                                            *
*                                                                               *
* RETURN VALUE: void                                                            *
*                                                                               *
********************************************************************************/

static float rand_n;
void seed(const float s)
{
    rand_n = s;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: equals                                                         *
*                                                                               *
* PURPOSE: Normally distributed random numbers generator that uses a            *
*           congruent linear algorithm, with c=0, a=16807 and m=2147483647      *
*           returning -1 if failed,                                             *
*           the pointer to the value if successfull                             *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* none                                                                          *
*                                                                               *
* RETURN VALUE: float                                                           *
*                                                                               *
********************************************************************************/
float randn() 
{
    if (rand_n > 0) 
    {
        rand_n = rand_n * 16807;
        rand_n = fmodf(rand_n, 2147483647);
        return rand_n;
    }
    else 
    {
        perror("Not a valid seed");
        return -1;
    }
}
