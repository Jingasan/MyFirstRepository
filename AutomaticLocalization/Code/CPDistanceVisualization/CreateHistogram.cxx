/***** �Ή��_�Ԃ̋����̃q�X�g�O�����̍쐬 *****/
// �Ή��_�Ԃ̋������L�^���ꂽ�G�N�Z���t�@�C����ǂݍ��݁A�q�X�g�O�������쐬�E�o�͂���
// �� �q�X�g�O�����̕`��ɂ́A�G�N�Z�����g�p���邱��


/*** �C���N���[�h ***/

/* C */
#include <math.h>
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <limits.h>

/* C++ */
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <utility>

/* �ݒ�w�b�_�t�@�C�� */
#include "Configuration.h"


/*** ���O��Ԃ̐錾 ***/

/* C++ */
using namespace std;


/*** ���C�������֐� ***/
int main( int argc, char **argv ){


	/* �ϐ��̐錾 */
	FILE *inputFile;					// �t�@�C���^�ϐ�
	int index;							// �Ή��_�̃C���f�b�N�X
	float distance;						// �Ή��_�Ԃ̋���
	float distanceInt;					// �Ή��_�Ԃ̋���( int �^�� MULTIPLY_NUM �{�� )
	vector< float > distances;			// �Ή��_�Ԃ̋����̔z��
	vector< int > distancesInt;			// �Ή��_�Ԃ̋����̔z��( int �^�� MULTIPLY_NUM �{�� )
	float meanCPDistance = 0;			// �Ή��_�Ԃ̋����̕��ϒl
	float rmse = 0;						// RMSE( root mean squared error )�F���ϓ��덷�̕�����
	float maxCPDistance = -FLT_MAX;		// �Ή��_�Ԃ̋����̍ő�l
	float minCPDistance = FLT_MAX;		// �Ή��_�Ԃ̋����̍ŏ��l 
	int maxCPDistanceInt = -INT_MAX;	// �Ή��_�Ԃ̋����̍ő�l( int �^ )
	int minCPDistanceInt = INT_MAX;		// �Ή��_�Ԃ̋����̍ŏ��l( int �^ )
	int binNum;							// �q�X�g�O�����̃r���̐�( ���� )
	int binMaxSize = -INT_MAX;			// �q�X�g�O�����̃r���̍ő卂( �c�� )


	/*** �v���O�����J�n�̃R�[�� ***/
	cout << "�y �v���O�����J�n �z" << endl;


	/*** �Ή��_�Ԃ̋����̃G�N�Z���f�[�^�̓ǂݍ��� ***/
	cout << ">> �Ή��_�Ԃ̋����f�[�^�̓ǂݍ��ݒ�" << "\r";

	/* �t�@�C���ǂݍ��ݎ��s�� */
	if( ( inputFile = fopen( CPDistanceFileName, "r" ) ) == NULL ){
		cout << "�t�@�C�� " << CPDistanceFileName << " ���J���܂���" << endl;
		exit( 0 );
	}

	/* �������̊m�� */
	distances.resize( 0 );	// �Ή��_�Ԃ̋������i�[����z��

	/* �t�@�C�����e�̓ǂݍ��� */
	while( fscanf( inputFile, "%d,%f", &index, &distance ) != EOF ){

		/* ���̏����ʂŐ؂藎�Ƃ����Ή��_�Ԃ̋����l�̎Z�o */
		distanceInt = (int)( distance * MULTIPLY_NUM );
		
		/* �X�^�b�N�z��Ɋi�[ */
		distances.push_back( distance );		// �Ή��_�Ԃ̋����l
		distancesInt.push_back( distanceInt );	// �Ή��_�Ԃ̋����l( int �^�� MULTIPLY_NUM �{�� )

		/* �Ή��_�Ԃ̋����̘a�Ɠ��a�̎Z�o */
		meanCPDistance += distance;
		rmse += distance * distance;

		/* �Ή��_�Ԃ̋����̍ő�l�ƍŏ��l�̎Z�o */
		if( maxCPDistance < distance ) maxCPDistance = distance;
		if( distance < minCPDistance ) minCPDistance = distance;
		if( maxCPDistanceInt < distanceInt ) maxCPDistanceInt = distanceInt;
		if( distanceInt < minCPDistanceInt ) minCPDistanceInt = distanceInt;

	}
	
	/* �������̉�� */
	fclose( inputFile );
	cout << ">> �Ή��_�Ԃ̋����f�[�^�̓ǂݍ��݊���" << endl;
	cout << distancesInt.size() << endl;


	/*** �Ή��_�Ԃ̋����̕��ϒl�� RMSE �̎Z�o ***/
	meanCPDistance = meanCPDistance / distances.size();	// ���ϒl
	rmse = sqrt( rmse / distances.size() );				// RMSE


	/*** �q�X�g�O�����̍쐬 ***/

	/* �q�X�g�O�����̃r�����̎Z�o */
	binNum = maxCPDistanceInt - minCPDistanceInt + 1;

	/* �r���̃������m�ۂƏ����� */
	int *bin = new int[ binNum ];
	for( int i = 0; i < binNum; i++ ) bin[ i ] = 0;

	/* �q�X�g�O�������\������e�r���̍쐬 */
	for( int i = 0; i < distancesInt.size(); i++ ){

		/* �Ή��_�Ԃ̋����̃J�E���g */
		bin[ distancesInt[ i ] - minCPDistanceInt ]++;
		
		//cout << "Index: " << i << ", Distance: " << distances[ i ] << endl;
		//cout << "Index: " << i << ", DistanceInt: " << distancesInt[ i ] << endl;
	}

	cout << "mean_CPDistance: " << meanCPDistance << endl;
	cout << "RMSE_CPDistance: " << rmse << endl;
	cout << "max_CPDistance: " << maxCPDistance << endl;
	cout << "min_CPDistance: " << minCPDistance << endl;
	cout << "max_CPDistance " << MULTIPLY_NUM << " �{ : " << maxCPDistanceInt << endl;
	cout << "min_CPDistance " << MULTIPLY_NUM << " �{ : " << minCPDistanceInt << endl;
	cout << "binNum: " << binNum << endl;
	
	/* �r���̍����̍ő�l�̎Z�o */
	for( int i = 0; i < binNum; i++ ){

		if( binMaxSize < bin[ i ] ) binMaxSize = bin[ i ];

		/* �Ή��_�Ԃ̋����Ƃ��̏o���񐔂̃R�}���h���C���o�� */
		//cout << "BinNumber: " << i + 1 << ", Distance: " << minCPDistanceInt + i << ", Count: " << bin[ i ] << endl;
	}


	/*** �q�X�g�O�����̃r���̃G�N�Z���t�@�C���o�� ***/

	/* �t�@�C���X�g���[���ϐ��̒�` */
	ofstream output;

	/* �o�̓t�@�C���p�X�̐ݒ� */
	output.open( HistogramFileName );

	/* �G�N�Z���o�� */
	//for( int i = 0; i < binNum; i++ ) output << i << "," << bin[ i ] << endl;
	for( int i = 0; i < binNum; i++ ) output << minCPDistanceInt + i << "," << bin[ i ] << endl;

	/* �������̉�� */
	output.close();


	/*** �v���O�����I���̃R�[�� ***/
	cout << "�y �v���O�����I�� �z" << endl;


	/* �������̉�� */
	delete bin;								// �q�X�g�O�����̃r���p�z��


	return 0;
}