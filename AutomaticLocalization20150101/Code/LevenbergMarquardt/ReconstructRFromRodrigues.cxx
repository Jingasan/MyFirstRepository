/***** Rodrigues の回転ベクトルからの回転行列の復元 *****/
// LevMarOptimization 関数内で使用


/*** インクルードファイル ***/

/* 設定用ヘッダファイル */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** 名前空間の宣言 ***/

/* C++ */
using namespace std;


/*** Rodrigues の回転ベクトルからの回転行列の復元 ***/
Eigen::Matrix< double, 3, 3 > ReconstructRFromRodrigues(
	Eigen::Vector3d* RodriguesVector	// Rodrigues を用いた回転表現 r = [ r1, r2, r3 ]
){

	/*** 変数の宣言と初期化 ***/

	/* Rodrigues 変換後の3×1回転ベクトルにより構成される3×3行列 [r]x */
	Eigen::Matrix< double, 3, 3 > RodriguesMatrix;
	RodriguesMatrix( 0, 0 ) = 0; RodriguesMatrix( 0, 1 ) = - ( *RodriguesVector )( 2 ); RodriguesMatrix( 0, 2 ) = ( *RodriguesVector )( 1 );
	RodriguesMatrix( 1, 0 ) = ( *RodriguesVector )( 2 ); RodriguesMatrix( 1, 1 ) = 0; RodriguesMatrix( 1, 2 ) = - ( *RodriguesVector )( 0 );
	RodriguesMatrix( 2, 0 ) = - ( *RodriguesVector )( 1 ); RodriguesMatrix( 2, 1 ) = ( *RodriguesVector )( 0 ); RodriguesMatrix( 2, 2 ) = 0;
	
	/* 回転量θ */
	double theta = ( *RodriguesVector ).norm();	// θ = || r ||

	/* 3×3の単位行列の作成 */
	Eigen::Matrix< double, 3, 3 > I;
	I = Eigen::MatrixXd::Identity( 3, 3 );

	/* 回転行列 R の復元 */
	Eigen::Matrix< double, 3, 3 > R;
	R = I + ( sin( theta ) / theta ) * RodriguesMatrix + ( ( 1 - cos( theta ) ) / pow( theta, 2 ) ) * RodriguesMatrix * RodriguesMatrix;

	/* 返り値 */
	return R;	// 回転行列
}