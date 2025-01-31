#pragma once
/******************************************************************************/
/* File   : infClientSwcApplDigitalTwinMotorPMSM.hpp                          */
/* Author : Nagaraja HULIYAPURADA-MATA                                        */
/* Date   : 01.02.1982                                                        */
/******************************************************************************/

/******************************************************************************/
/* #INCLUDES                                                                  */
/******************************************************************************/
#include "fixedpoint.hpp"

/******************************************************************************/
/* #DEFINES                                                                   */
/******************************************************************************/

/******************************************************************************/
/* MACROS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* TYPEDEFS                                                                   */
/******************************************************************************/
typedef struct{
   s42p21 Tm;
   s42p21 Va;
   s42p21 Vb;
   s42p21 Vc;
}Type_infClientSwcApplDigitalTwinMotorPMSM_stInputs;

typedef struct{
   s42p21 Ia;
   s42p21 Ib;
   s42p21 Ic;
}Type_infClientSwcApplDigitalTwinMotorPMSM_stOutputs;

typedef struct{
   s42p21 Iq;
   s42p21 Id;
   s42p21 Wm;
   s42p21 We;
   s42p21 Theta_e;
   s42p21 Theta_e_cos;
   s42p21 Theta_e_sin;
}Type_SwcApplDigitalTwinMotorPMSM_stIntermediate;

typedef struct{
   s42p21 Fqx3;
   s42p21 Fdxsqrt3;
   s42p21 Vq;
   s42p21 Vd;
   s42p21 Dq;
   s42p21 Dd;
   s42p21 Te;
   s42p21 DWm;
   s42p21 Theta_m; // TBD: Remove
   s42p21 Ialpha;
   s42p21 Ibeta;
   s42p21 Ibetaxsqrt3p0;
}Type_SwcApplDigitalTwinMotorPMSM_stLocals;

class Type_infClientSwcApplDigitalTwinMotorPMSM{
   public:
      virtual void InitFunction(
            const Type_infClientSwcApplDigitalTwinMotorPMSM_stInputs*  lpcstInputs
         ,        Type_SwcApplDigitalTwinMotorPMSM_stIntermediate*     lpstIntermediate
         ,        Type_infClientSwcApplDigitalTwinMotorPMSM_stOutputs* lpstOutputs
      ) = 0;
      virtual void MainFunction   (void) = 0;
      virtual void DeInitFunction (void) = 0;
};

/******************************************************************************/
/* CONSTS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* PARAMS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* OBJECTS                                                                    */
/******************************************************************************/
extern Type_infClientSwcApplDigitalTwinMotorPMSM* pinfClientSwcApplDigitalTwinMotorPMSM;

/******************************************************************************/
/* FUNCTIONS                                                                  */
/******************************************************************************/

/******************************************************************************/
/* EOF                                                                        */
/******************************************************************************/

