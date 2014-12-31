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
void PartialDerivativeForRodrigues(
	Eigen::Vector3d* RodriguesVector,	// Rodrigues を用いた回転表現 r = [ r1, r2, r3 ]
	Eigen::Matrix3d* dR
){

	/* 変数の定義 */
	double theta;					// 回転の3次元ベクトルのノルム
	double theta2, theta3, theta4;	// theta のX乗
	Eigen::Matrix3d rx;				// 行列 rx
	Eigen::Matrix3d rx2;			// 行列 rx の2乗
	double cosTheta, sinTheta;		// コサイン，サインの値
	Eigen::Matrix3d M[ 3 ];			// rxをrの要素でそれぞれ微分したときの行列(固定された値)
	Eigen::Matrix3d tmp;			// 一時的に確保する行列
	Eigen::Matrix3d r;				// Rodrigues の回転ベクトル
	
	/* Rodrigues の回転ベクトル */
	r( 0 ) = (*RodriguesVector)( 0 );
	r( 1 ) = (*RodriguesVector)( 1 );
	r( 2 ) = (*RodriguesVector)( 2 );

	/* θの算出 */
	theta = r.norm();

	/* θが 0 の場合 ⇒ 単位行列 */
	if( theta == 0 ){
		for( int i = 0; i < 3; i++ ) dR[ i ] << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	}else{

		/* θの乗算 */
		theta2 = theta * theta;		// 2乗
		theta3 = theta2 * theta;	// 3乗
		theta4 = theta3 * theta;	// 4乗

		/* 行列 rx の算出 */
		rx( 0, 0 ) = 0;	rx( 0, 1 ) = -1.0 * r( 2 ); rx( 0, 2 ) = r( 1 );
		rx( 1, 0 ) = r( 2 ); rx( 1, 1 ) = 0; rx( 1, 2 ) = -1.0 * r( 0 );
		rx( 2, 0 ) = -1.0 * r( 1 ); rx( 2, 1 ) = r( 0 ); rx( 2, 2 ) = 0;

		/* 行列 rx の二乗の算出 */
		rx2 = rx * rx;

		/* 行列 M の作成 */
		M[ 0 ] << 0, 0, 0, 0, 0, -1, 0, 1, 0; 
		M[ 1 ] << 0, 0, 1, 0, 0, 0, -1, 0, 0; 
		M[ 2 ] << 0, -1, 0, 1, 0, 0, 0, 0, 0; 

		/* sinθ，cosθ の値を算出 */
		cosTheta = cos( theta );
		sinTheta = sin( theta );

		/* 回転行列の偏微分：回転ベクトル３要素でそれぞれ微分 */
		for( int i = 0; i < 3; i++ ){
			tmp = M[ i ] * rx + rx * M[ i ];
			dR[ i ] = ( ( r( i ) * cosTheta / theta2 ) - ( r( i ) * sinTheta / theta3 ) ) * rx.array()
				+ ( sinTheta / theta ) * M[ i ].array()
				+ ( ( r( i ) * sinTheta / theta3 ) - ( ( 2 * r( i ) * ( 1.0 - cosTheta ) ) / theta4 ) ) * rx2.array()
				+ ( ( 1.0 - cosTheta ) / theta2 ) * tmp.array();
		}
	}

	
}