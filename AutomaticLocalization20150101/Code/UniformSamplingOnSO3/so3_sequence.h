/***** ��]��Ԃ̋ϓ��ȃT���v�����O( �w�b�_�t�@�C�� ) *****/


/*** �萔�̒�` ***/
#define M_PI 3.141592	// �~����


/*** �C���N���[�h�t�@�C�� ***/
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
//#include <stdbool.h>


/*** ���O��Ԃ̐錾 ***/
using namespace std;


/*** �֐��̒�` ***/
void pix2ang_nest( long, long, double*, double* );
void mk_pix2xy( int *, int* );
vector<double> find_point( int, long, long, long, double );
bool hopf2quat( vector< vector<double> > );
