/***** 設定ヘッダファイル *****/


/*** フラグの管理 ***/

/* VoxelGrid による点群の間引き */
//#define REDUCTION


/*** 定数の定義 ***/

/* Canny フィルタ関連の初期化値 */
#define VARIANCE 2.0		// ガウシアンの窓サイズ
#define UPPERTHRESHOLD 0.0	// 上限閾値
#define LOWERTHRESHOLD 0.0	// 下限閾値

/* VoxelGrid による点群の間引き関連の初期化値 */
#define VOXELGRIDSIZE 10.0	// ボクセルグリッドサイズ[ 単位：m ]
