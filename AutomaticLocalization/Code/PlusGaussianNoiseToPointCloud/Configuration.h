/***** 設定ヘッダファイル *****/


/*** フラグの管理 ***/
//#define COMMENT	// コメントのコマンドライン出力 ON：出力, OFF：非出力
#define VIEWER	// ノイズ付加後の点群の可視化 ON: 可視化, OFF：非可視化


/*** 定数の定義 ***/

/* ノイズのスケーリング */
#define NOISESCALE 1	// ガウシアンの標準偏差幅のスケール値

/* 入力ファイル */
const char InputGaussianNoiseFileName[] = "Input/GaussianNoise/noise10000.csv";	// ガウシアンノイズのファイル名
