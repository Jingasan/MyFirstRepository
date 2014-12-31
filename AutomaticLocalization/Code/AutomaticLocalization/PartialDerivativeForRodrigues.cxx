/***** 回転行列の偏微分 *****/
// LevMarOptimization 関数内で使用


/*** インクルードファイル ***/

/* 設定用ヘッダファイル */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** 名前空間の宣言 ***/

/* C++ */
using namespace std;


/*** 回転行列の r1, r2, r3 成分による偏微分 ***/
// rNum = 0 の場合 r1 で微分, rNum = 1 の場合 r2 で微分, rNum = 2 の場合 r3 で微分された回転行列が得られる
Eigen::Matrix< double, 3, 3 > PartialDerivativeForRodrigues(
	int rNum,							// 偏微分する回転成分番号 ⇒ 0：r1, 1：r2, 2：r3
	Eigen::Vector3d* RodriguesVector	// Rodrigues を用いた回転表現 r = [ r1, r2, r3 ]
){

	/*** 変数の宣言と初期化 ***/

	/* Rodrigues 変換後の3×1回転ベクトルにより構成される3×3行列 [r]x */
	Eigen::Matrix< double, 3, 3 > RodriguesMatrix;
	RodriguesMatrix( 0, 0 ) = 0; RodriguesMatrix( 0, 1 ) = - ( *RodriguesVector )( 2 ); RodriguesMatrix( 0, 2 ) = ( *RodriguesVector )( 1 );
	RodriguesMatrix( 1, 0 ) = ( *RodriguesVector )( 2 ); RodriguesMatrix( 1, 1 ) = 0; RodriguesMatrix( 1, 2 ) = - ( *RodriguesVector )( 0 );
	RodriguesMatrix( 2, 0 ) = - ( *RodriguesVector )( 1 ); RodriguesMatrix( 2, 1 ) = ( *RodriguesVector )( 0 ); RodriguesMatrix( 2, 2 ) = 0;

	/* Rodrigues 変換後の3×1回転ベクトルにより構成される3×3行列 [r]x を r1, r2, r3 で偏微分した3×3行列 */
	Eigen::Matrix< double, 3, 3 > M[ 3 ];

	/* r1 で微分した場合の [r]x */
	M[ 0 ]( 0, 0 ) = 0; M[ 0 ]( 0, 1 ) = 0; M[ 0 ]( 0, 2 ) = 0;
	M[ 0 ]( 1, 0 ) = 0; M[ 0 ]( 1, 1 ) = 0; M[ 0 ]( 1, 2 ) = -1;
	M[ 0 ]( 2, 0 ) = 0; M[ 0 ]( 2, 1 ) = 1; M[ 0 ]( 2, 2 ) = 0;
	
	/* r2 で微分した場合の [r]x */
	M[ 1 ]( 0, 0 ) = 0; M[ 1 ]( 0, 1 ) = 0; M[ 1 ]( 0, 2 ) = 1;
	M[ 1 ]( 1, 0 ) = 0; M[ 1 ]( 1, 1 ) = 0; M[ 1 ]( 1, 2 ) = 0;
	M[ 1 ]( 2, 0 ) = -1; M[ 1 ]( 2, 1 ) = 0; M[ 1 ]( 2, 2 ) = 0;
	
	/* r3 で微分した場合の [r]x */
	M[ 2 ]( 0, 0 ) = 0; M[ 2 ]( 0, 1 ) = -1; M[ 2 ]( 0, 2 ) = 0;
	M[ 2 ]( 1, 0 ) = 1; M[ 2 ]( 1, 1 ) = 0; M[ 2 ]( 1, 2 ) = 0;
	M[ 2 ]( 2, 0 ) = 0; M[ 2 ]( 2, 1 ) = 0; M[ 2 ]( 2, 2 ) = 0;
	
	/* 回転量θ */
	double theta = ( *RodriguesVector ).norm();	// θ = || r ||


	/*** 偏微分された回転行列 ***/
	
	/* 微分された回転行列の各項 */
	Eigen::Matrix< double, 3, 3 > term1 = ( ( *RodriguesVector )( rNum ) * cos( theta ) / pow( theta, 2 ) ) * RodriguesMatrix;
	Eigen::Matrix< double, 3, 3 > term2 = ( ( *RodriguesVector )( rNum ) * sin( theta ) / pow( theta, 3 ) ) * RodriguesMatrix;
	Eigen::Matrix< double, 3, 3 > term3 = ( sin( theta ) / theta ) * M[ rNum ];
	Eigen::Matrix< double, 3, 3 > term4 = ( ( *RodriguesVector )( rNum ) * sin( theta ) / pow( theta, 3 ) ) * RodriguesMatrix * RodriguesMatrix;
	Eigen::Matrix< double, 3, 3 > term5 = ( ( 2 * ( *RodriguesVector )( rNum ) ) * ( 1 - cos( theta ) ) / pow( theta, 4 ) ) * RodriguesMatrix * RodriguesMatrix;
	Eigen::Matrix< double, 3, 3 > term6 = ( ( 1 - cos( theta ) ) / pow( theta, 2 ) ) * ( ( M[ rNum ] * RodriguesMatrix ) + ( RodriguesMatrix * M[ rNum ] ) );

	/* 微分結果となる回転行列 : rNum = 0 の場合 r1 で微分, rNum = 1 の場合 r2 で微分, rNum = 2 の場合 r3 で微分された回転行列が得られる */
	Eigen::Matrix< double, 3, 3 > dR = term1 - term2 + term3 + term4 - term5 + term6;


	/* 返り値 */
	return dR;	// 微分結果となる回転行列
}