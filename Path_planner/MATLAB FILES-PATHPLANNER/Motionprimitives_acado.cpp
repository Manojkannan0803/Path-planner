/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    TIME autotime;
    DifferentialState x1;
    DifferentialState y1;
    DifferentialState theta1;
    DifferentialState gamma;
    DifferentialState delta;
    DifferentialState omega;
    Control u;
    Parameter T; 
    DifferentialEquation acadodata_f1(0, T);
    acadodata_f1 << dot(x1) == (1.00000000000000000000e+00-7.89473684210526271965e-02*tan(delta)*tan(gamma))*cos(gamma)*cos(theta1);
    acadodata_f1 << dot(y1) == (1.00000000000000000000e+00-7.89473684210526271965e-02*tan(delta)*tan(gamma))*cos(gamma)*sin(theta1);
    acadodata_f1 << dot(theta1) == (7.89473684210526271965e-02*cos(gamma)*tan(delta)+sin(gamma))*1.17994100294985249100e-01;
    acadodata_f1 << dot(gamma) == (-1.17994100294985249100e-01*sin(gamma)+2.63157894736842090655e-01*tan(delta)-9.31532370749883417704e-03*cos(gamma)*tan(delta));
    acadodata_f1 << dot(delta) == omega;
    acadodata_f1 << dot(omega) == u;

    OCP ocp1(0, T);
    ocp1.minimizeMayerTerm(T);
    ocp1.subjectTo(acadodata_f1);
    ocp1.subjectTo(AT_START, x1 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, y1 == 1.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, theta1 == 3.14159265358979311600e+00);
    ocp1.subjectTo(AT_START, gamma == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, delta == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, omega == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, u == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_END, y1 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_END, theta1 == 3.14159265358979311600e+00);
    ocp1.subjectTo(AT_END, gamma == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_END, delta == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_END, omega == 0.00000000000000000000e+00);
    ocp1.subjectTo((-1.39626340159546358244e+00) <= gamma <= 1.39626340159546358244e+00);
    ocp1.subjectTo((-7.85398163397448278999e-01) <= delta <= 7.85398163397448278999e-01);
    ocp1.subjectTo((-1.30000000000000004441e-01) <= omega <= 1.30000000000000004441e-01);
    ocp1.subjectTo((-4.00000000000000000000e+00) <= u <= 4.00000000000000000000e+00);
    ocp1.subjectTo(1.00000000000000000000e+01 <= T <= 4.50000000000000000000e+01);


    OptimizationAlgorithm algo1(ocp1);
    algo1.set( DISCRETIZATION_TYPE, COLLOCATION );
    algo1.set( MAX_NUM_ITERATIONS, 300 );
    algo1.set( KKT_TOLERANCE, 1 );
    algo1.set( INTEGRATOR_TYPE, INT_BDF );
    algo1.set( INTEGRATOR_TOLERANCE, 1.000000E-01 );
    algo1.set( ABSOLUTE_TOLERANCE, 1.000000E-01 );
    algo1.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
    returnValue returnvalue = algo1.solve();

    VariablesGrid out_states; 
    VariablesGrid out_parameters; 
    VariablesGrid out_controls; 
    VariablesGrid out_disturbances; 
    VariablesGrid out_algstates; 
    algo1.getDifferentialStates(out_states);
    algo1.getControls(out_controls);
    algo1.getParameters(out_parameters);
    const char* outputFieldNames[] = {"STATES", "CONTROLS", "PARAMETERS", "DISTURBANCES", "ALGEBRAICSTATES", "CONVERGENCE_ACHIEVED"}; 
    plhs[0] = mxCreateStructMatrix( 1,1,6,outputFieldNames ); 
    mxArray *OutS = NULL;
    double  *outS = NULL;
    OutS = mxCreateDoubleMatrix( out_states.getNumPoints(),1+out_states.getNumValues(),mxREAL ); 
    outS = mxGetPr( OutS );
    for( int i=0; i<out_states.getNumPoints(); ++i ){ 
      outS[0*out_states.getNumPoints() + i] = out_states.getTime(i); 
      for( int j=0; j<out_states.getNumValues(); ++j ){ 
        outS[(1+j)*out_states.getNumPoints() + i] = out_states(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"STATES",OutS );
    mxArray *OutC = NULL;
    double  *outC = NULL;
    OutC = mxCreateDoubleMatrix( out_controls.getNumPoints(),1+out_controls.getNumValues(),mxREAL ); 
    outC = mxGetPr( OutC );
    for( int i=0; i<out_controls.getNumPoints(); ++i ){ 
      outC[0*out_controls.getNumPoints() + i] = out_controls.getTime(i); 
      for( int j=0; j<out_controls.getNumValues(); ++j ){ 
        outC[(1+j)*out_controls.getNumPoints() + i] = out_controls(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"CONTROLS",OutC );
    mxArray *OutP = NULL;
    double  *outP = NULL;
    OutP = mxCreateDoubleMatrix( out_parameters.getNumPoints(),1+out_parameters.getNumValues(),mxREAL ); 
    outP = mxGetPr( OutP );
    for( int i=0; i<out_parameters.getNumPoints(); ++i ){ 
      outP[0*out_parameters.getNumPoints() + i] = out_parameters.getTime(i); 
      for( int j=0; j<out_parameters.getNumValues(); ++j ){ 
        outP[(1+j)*out_parameters.getNumPoints() + i] = out_parameters(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"PARAMETERS",OutP );
    mxArray *OutW = NULL;
    double  *outW = NULL;
    OutW = mxCreateDoubleMatrix( out_disturbances.getNumPoints(),1+out_disturbances.getNumValues(),mxREAL ); 
    outW = mxGetPr( OutW );
    for( int i=0; i<out_disturbances.getNumPoints(); ++i ){ 
      outW[0*out_disturbances.getNumPoints() + i] = out_disturbances.getTime(i); 
      for( int j=0; j<out_disturbances.getNumValues(); ++j ){ 
        outW[(1+j)*out_disturbances.getNumPoints() + i] = out_disturbances(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"DISTURBANCES",OutW );
    mxArray *OutZ = NULL;
    double  *outZ = NULL;
    OutZ = mxCreateDoubleMatrix( out_algstates.getNumPoints(),1+out_algstates.getNumValues(),mxREAL ); 
    outZ = mxGetPr( OutZ );
    for( int i=0; i<out_algstates.getNumPoints(); ++i ){ 
      outZ[0*out_algstates.getNumPoints() + i] = out_algstates.getTime(i); 
      for( int j=0; j<out_algstates.getNumValues(); ++j ){ 
        outZ[(1+j)*out_algstates.getNumPoints() + i] = out_algstates(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"ALGEBRAICSTATES",OutZ );
    mxArray *OutConv = NULL;
    if ( returnvalue == SUCCESSFUL_RETURN ) { OutConv = mxCreateDoubleScalar( 1 ); }else{ OutConv = mxCreateDoubleScalar( 0 ); } 
    mxSetField( plhs[0],0,"CONVERGENCE_ACHIEVED",OutConv );


    clearAllStaticCounters( ); 
 
} 

