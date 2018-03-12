%%%%%%%%%%%%%%%%%% University of São Paulo - USP %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Author: Leonardo Borges Farçoni                       %%%%%%%%%%%%%%
%%%%%% e-mail: leonardo.farconi@gmail.com                    %%%%%%%%%%%%%%
%%%%%% Professor Advisor: Marco H. Terra and Roberto Inoue   %%%%%%%%%%%%%%
%%%%%% E-mail: terra@sc.usp.br and rsinoue@ufscar.br         %%%%%%%%%%%%%%
%%%%%% Date: July 10th, 2017                                 %%%%%%%%%%%%%%
%%%%%% Based on first code version by Isabella Cristina Souza%%%%%%%%%%%%%%
%%%%%% Faria (isamoreno2009@gmail.com) from January 20,2015  %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Revision 1:                                           %%%%%%%%%%%%%%

clear all
close all
clc
path(path,'.\trajectory')
path(path,'.\model')
path(path,'.\model\multiSim')
path(path,'.\interfaces')
path(path,'.\figures')
path(path,'.\control')
%% Graphical User Interface
gui = guidata(gui_quad)

%% GUI settings
configureGUI

%% Aircraft definition
configureAircraft

%% Control variables
controller.type = 'rlqr';
configureController

%% Initial Conditions
loadInitialConditions



