clear; clc; close all;
set(0,'DefaultFigureWindowStyle','docked')

syms a11 a12 a13 a14
syms a21 a22 a23 a24
syms a31 a32 a33 a34
syms a41 a42 a43 a44

A=[a11 a12 a13 a14;
   a21 a22 a23 a24;
   a31 a32 a33 a34;
   a41 a42 a43 a44];

det(A)