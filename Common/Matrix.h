#ifndef Matrix_h
#define Matrix_h

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <cassert>
#include <iostream>

using namespace std;

#if !defined DELETE_OPS
#define DELETE_OPS
#define CLEAN_DELETE(p) if(p) delete p
#define CLEAN_ARRAY_DELETE(p) if(p) delete [] p
#endif


template <class t, int n> class Vector;
template <class Type, int N>
class Matrix
{

protected:
	int rows, cols;
	Type **data;

	int pivot(int row){
		int k = row;
		double amax, temp;

		amax = -1;
		for(int i=row;i<rows;i++){
			if((temp=abs(data[i][row]))> amax && temp!=0.0){
				amax = temp;
				k = i;
			}
		}

		if(data[k][row] == 0) 
			return -1;
		if(k != row){
			Type *rowptr = data[k];
			data[k] = data[row];
			data[row] = rowptr;
			return k;
		}

		return 0;
	}

public:
	Matrix() : rows(N), cols(N), data(NULL){ 
		data = new Type*[rows];
		for(int r=0;r<rows;r++){
			data[r] = new Type[cols];
			memset(data[r], 0, cols * sizeof(Type));
		}

		int s = rows <cols ? rows : cols;
		for(int r=0;r<s;r++)
			data[r][r] = 1;
	}

	Matrix(const Matrix<Type,N> &M){
		rows = M.rows;
		cols = M.cols;

		data = new Type *[rows];
		int copySize = sizeof(Type) * cols;

		for(int r=0;r<rows;r++){
			data[r] = new Type[cols];
			memcpy(data[r], M.data[r], copySize);
		}
	}

	virtual ~Matrix(){
		if(data){
			for(int r=0;r<rows;r++){
				CLEAN_ARRAY_DELETE(data[r]);
			}
			CLEAN_ARRAY_DELETE(data);
		}
	}

	Type **Data(){ return data; }

	void identity(){
		int s = rows <cols ? rows : cols;
		int copySize = cols * sizeof(Type);
		for(int i = 0; i <rows; i++)
			memset(data[i], 0, copySize);
		for(int i = 0; i <s; i++)
			data[i][i] = 1;
	}

	void clean(){
		if(data){
			for(int r=0;r<rows;r++){
				CLEAN_ARRAY_DELETE(data[r]);
			}
			CLEAN_ARRAY_DELETE(data);
		}
	}

	void clear(){
		for(int r=0;r<rows;r++)
			memset(data[r], 0, cols * sizeof(Type));
	}

	int Rows() const{ return rows; }
	int Cols() const{ return cols; }

	void operator=(const Matrix<Type,N> &M){
		if(data)
		{
			for(int r = 0; r <rows; r++)
			{
				CLEAN_DELETE(data[r] );
			}
			CLEAN_DELETE(data);
		}

		rows = M.rows;
		cols = M.cols;

		data = new Type *[rows];
		for(int r=0;r<rows;r++){
			data[r] = new Type[cols];
			memcpy(data[r], M.data[r], sizeof(Type) * cols);
		}
	}

	Type*& operator[] (int r){
		assert(r>= 0 && r <rows);
		return data[r];
	}
	
	Type *operator[] (int r) const{
		assert(r>= 0 && r <rows);
		return data[r];
	}

	public:
	/************************* FRIEND FUNCTIONS FOR MATRICES *****************************/
	friend Matrix<Type,N> operator + (Matrix<Type,N> &L, Matrix<Type,N> &R){
		assert((L.rows == R.rows) && (L.cols == R.cols));
		int rows = L.rows;
		int cols = R.cols;

		Matrix<Type,N> M(rows, cols);
		for(int r=0;r<rows;r++)
			for(int c=0;c<cols;c++)
				M[r][c] = L.data[r][c] + R.data[r][c];

		return M;
	}

	friend Matrix<Type,N> operator*(const Matrix<Type,N> &L, const Matrix<Type,N> &R){
		assert(L.cols == R.rows);

		int rows = L.rows;
		int cols = R.cols;

		int d = L.cols;

		Matrix<Type,N> M; 
		for(int r=0;r<rows;r++) {
			for(int c=0;c<cols;c++){
				M[r][c] = 0;

				for(int k=0;k<d;k++)
					M[r][c] += L.data[r][k] * R.data[k][c];
			}
		}

		return M;
	}

	friend Matrix<Type,N> operator*(Type alpha, const Matrix<Type,N> &R)
	{
		int rows = R.rows;
		int cols = R.cols;

		Matrix<Type,N> M(rows, cols);
		for(int r=0;r<rows;r++)
			for(int c=0;c<cols;c++)
				M[r][c] = alpha * R.data[r][c];

		return M;
	}

	friend Matrix<Type,N> operator*(const Matrix<Type,N> &L, Type alpha){
		int rows = L.rows;
		int cols = L.cols;

		Matrix<Type,N> M(rows, cols);
		for(int r=0;r<rows;r++)
			for(int c=0;c<cols;c++)
				M[r][c] = alpha * L.data[r][c];

		return M;
	}

	friend Matrix<Type,N> operator-(const Matrix<Type,N> &R){
		int rows = R.rows;
		int cols = R.cols;

		Matrix<Type,N> M(rows, cols);
		for(int r=0;r<rows;r++)	
			for(int c=0;c<cols;c++)
				M[r][c] = -R.data[r][c];

		return M;
	}

	friend Matrix<Type,N> operator-(const Matrix<Type,N> &L, const Matrix<Type,N> &R)
	{
		assert((L.rows == R.rows) && (L.cols == R.rows));
		int rows = L.rows;
		int cols = R.cols;

		Matrix<Type,N> M(rows, cols);
		for(int r=0;r<rows;r++)
			for(int c=0;c<cols;c++)
				M[r][c] = L.data[r][c] - R.data[r][c];

		return M;
	}

	friend Matrix<Type,N> transpose(Matrix<Type,N> M)
	{
		int rows = M.rows;
		int cols = M.cols;

		Matrix<Type,N> R(cols, rows);
		for(int r=0;r<rows;r++)
			for(int c=0;c<cols;c++)
				R[c][r] = M.data[r][c];
		return R;
	}

	friend Matrix<Type,N> operator!(const Matrix<Type,N> &T){
		int i, j, k;
		Type a1, a2,  *rowptr;

		Matrix<Type,N> M = T;
		Matrix<Type,N> MI; 
		MI.identity();

		if(M.rows != M.cols){
			cout <<"Inversion of a non-square matrix" <<endl;
		}
		else{
			for(k=0;k<M.rows;k++){
				int indx = M.pivot(k);
				if(indx == -1){
					cout <<"Inversion of a singular matrix" <<endl;
					return MI;
				}

				if(indx != 0){
					rowptr = MI[k];
					MI[k] = MI[indx];
					MI[indx] = rowptr;
				}

				a1 = M[k][k];
				for(j = 0; j <M.rows; j++){
					M[k][j] /= a1;
					MI[k][j] /= a1;
				}
				for(i = 0; i <M.rows; i++){
					if(i != k){
						a2 = M[i][k];
						for(j = 0; j <M.rows; j++){
							M[i][j] -= a2 * M[k][j];
							MI[i][j] -= a2 * MI[k][j];
						}
					}
				}
			}
		}

		return MI;
	}
};

template <class Type, int n>
class Vector{

protected:
	int size;
	Type data[n];

public:
	Vector() : size(n){ 
		memset(data, 0, size * sizeof(Type)); 
	}

	Vector(Type a, Type b){
		size = n; 
		data[0] = a; 
		data[1] = b; 
		if(size>2)
			data[2] = 1; 
	}


	Vector(Type a, Type b, Type c){
		size = n; 
		data[0] = a; 
		data[1] = b; 
		data[2] = c; 
		if(size>3)
			data[3] = 1; 
	}

	Vector(Type a, Type b, Type c, Type d){
		size = n; 
		data[0] = a; 
		data[1] = b; 
		data[2] = c; 
		data[3] = d; 
	}

	Vector(const Vector<Type, n> &vec){
		size = vec.size;
		memcpy (data, vec.data, sizeof (Type) * size);

	}


	virtual void operator += (const Vector<Type, n>& vec){
		for(int i = 0; i <size; i++)
			data[i] += vec.data[i];
	}

	virtual void operator -= (const Vector<Type, n> &vec){
		for(int i = 0; i <size; i++)
			data[i] -= vec.data[i];
	}


	virtual void operator *= (Type alpha){
		for(int i = 0; i <size; i++)
			data[i] *= alpha;
	}

	virtual void operator /= (Type alpha){
		for(int i = 0; i <size; i++)
			data[i] /= alpha;
	}

	virtual int operator = (const Vector<Type, n> &vec){
		size = vec.size;
		memcpy (data, vec.data, sizeof (Type) * size);
		return 0;
	}

	virtual ~Vector()	{ }

	void print() const{
		for(int i = 0; i <n; i++)
			cout <<data[i] <<" ";
		printf("\b");
	}
	

	void Zero(){ memset(data, 0, sizeof(Type) * size); }

	int Size() const{ return size; }

	inline Type & operator[](int i){
		return data[i];
	}

	inline Type operator[](int i) const{ return data[i]; }

	friend class Matrix<Type, n>;
	

	friend Vector<Type, n> operator+(const Vector<Type, n> &L, const Vector<Type, n> &R){
		assert(L.Size() == R.Size());
		int s = L.size;
		Vector<Type, n> m;
		for(int i=0;i<s;i++)
			m[i] = L.data[i]+R.data[i]; 
		return m;
	}

	friend Vector<Type, n> operator-(const Vector<Type, n> &L, const Vector<Type, n> &R){
		assert((L.Size() == R.Size()));
		int s = L.size;
		Vector<Type, n> m; 
		for(int i=0;i<s;i++)
			m[i] = L.data[i] - R.data[i];

		return m;
	}

	friend Vector<Type, n> operator-(const Vector<Type, n> &L){
		int s = L.size;
		Vector<Type, n> M;
		for(int i=0;i<s;i++)
			M[i] = -L.data[i];
		return M;
	}

	friend Vector<Type, n> operator*(Type alpha, const Vector<Type, n> &R){
		int s = R.size;
		Vector<Type, n> M;
		for(int i=0;i<s;i++)
			M[i] = alpha * R.data[i];
		return M; 
	}
	
	friend Vector<Type, n> operator*(const Vector<Type, n> &L, Type alpha)	{
		int s = L.size;
		Vector<Type, n> M; 
		for(int i=0;i<s;i++)
			M[i] = alpha * L.data[i];
		return M;
	}

	friend Vector<Type, n> operator*(const Vector<Type, n> &L, const Matrix<Type, n> &R){
		int size = L.size;
		assert(size == R.Rows());

		int cols = R.Cols();

		Vector<Type, n> LR;
		for(int c=0;c<cols;c++)	{
			LR[c] = 0;
			for(int r=0;r<size;r++)
				LR[c] += L.data[r] * R[r][c];
		}

		return LR;
	}


	friend Vector<Type, n> operator*(const Matrix<Type, n> &L, const Vector<Type, n> &R){
		int size = R.size;
		int rows = L.Rows();

		Vector<Type, n> LR;
		
		for(int r=0;r<rows;r++)	{
			LR[r] = 0;
			for(int c=0;c<size;c++)
				LR[r] += R.data[c] * L[r][c];
		}

		return LR;
	}
/**/

	friend Type operator*(const Vector<Type, n> &L, const Vector<Type, n> &R){
		assert(L.size == R.size);

		Type res = 0;
		for(int i=0;i<L.size;i++)
			res += L.data[i] * R.data[i];
		return res;
	}

	friend Type mag(const Vector<Type, n> &vec){
		return sqrt(vec * vec);
	}

	friend Vector<Type, n> multiply(const Vector<Type, n> &u, const Vector<Type, n> &v){
		assert(u.size == v.size && v.size==4);
		
		Vector<Type, n> res;
		res[0] = u[0] * v[0];
		res[1] = u[1] * v[1];
		res[2] = u[2] * v[2];
		res[3] = u[3] * v[3];

		return res;
	}
	
	friend Vector<Type, n> cross(const Vector<Type, n> &u, const Vector<Type, n> &v){
		assert(u.size == v.size && v.size==4);
		Vector<Type, n> res;

		res[0] = u[1] * v[2] - u[2] * v[1];
		res[1] = u[2] * v[0] - u[0] * v[2];
		res[2] = u[0] * v[1] - u[1] * v[0];
		res[3] = 0; 

		return res;
	}

	void normalize()	{
		Type m = mag(*this);
		if(m != 0){
			for(int i=0;i<size;i++)
				data[i] /= m;
		}
	}
};

typedef Vector<double,4> Pt3; // use 3d vector for affine transform 
typedef Pt3 Vec3; 
typedef Vector<double,4> Color; 
typedef Matrix<double,4> Mat4; 


#endif // Matrix_h