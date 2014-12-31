/***** Rodrigues を用いた回転表現 r = [ r1, r2, r3 ] への変換 ( LevMarOptimization 関数内で使用 ) *****/
// LevMarOptimization 関数内で使用


/*** インクルードファイル ***/

/* 設定用ヘッダファイル */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** 名前空間の宣言 ***/

/* C++ */
using namespace std;


/*** 通常の回転行列 R から Rodrigues の回転表現 r = [ r1, r2, r3 ] への変換 ***/
Eigen::Vector3d ConvertRodrigues(
	Eigen::Matrix< double, 3, 3 >* preR	// Rodrigues 変換前の 3 * 3 回転行列
){

	/* 回転角 */
	 double theta = acos( ( ( ( *preR ).trace() - 1.0 ) / 2.0 ) );

	/* 正規化された回転軸 */
	Eigen::Vector3d NormalizedRaxis;
	Eigen::Vector3d a;
	a( 0 ) = ( *preR )( 2, 1 ) - ( *preR )( 1, 2 );
	a( 1 ) = ( *preR )( 0, 2 ) - ( *preR )( 2, 0 );
	a( 2 ) = ( *preR )( 1, 0 ) - ( *preR )( 0, 1 );
	NormalizedRaxis = a / a.norm();

	/* 回転軸 ( Rodrigues を用いた回転表現 r = [ r1, r2, r3 ] ) */
	Eigen::Vector3d Raxis;
	Raxis = NormalizedRaxis * theta;


	/* 返り値 */
	return Raxis;	// Rodrigues を用いた回転表現 r = [ r1, r2, r3 ]
}