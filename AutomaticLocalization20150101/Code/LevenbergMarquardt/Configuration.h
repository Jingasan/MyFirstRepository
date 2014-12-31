/***** 設定ヘッダファイル *****/


/*** フラグの定義 ***/



/*** 変数の定義 ***/
#define LAMBDA 0.001		// マーカート法におけるλの初期値
#define THRESHOLD_C	0.1		// 評価関数 C の閾値： | nextC - C | < THRESHOLD_C なら終了
#define THRESHOLD_A 0.00001	// パラメータベクトル A の閾値： | nextA - A | < THRESHOLD_A なら終了
#define LOOP_MAX 1000		// 最大ループ回数

/* 出力ファイルパス */
const char OutputFileName[] = "Output/LevenbergMarquardt/mergedPointCloud.ply";
