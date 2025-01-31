#pragma once
/******************************************************************************/
/* File   : infClientSwcApplDigitalTwinMotorPMSM.hpp                          */
/* Author : Nagaraja HULIYAPURADA-MATA                                        */
/* Date   : 01.02.1982                                                        */
/******************************************************************************/

/******************************************************************************/
/* #INCLUDES                                                                  */
/******************************************************************************/

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
   double Tm;
   double Va;
   double Vb;
   double Vc;
}Type_infClientSwcApplDigitalTwinMotorPMSM_stInputs;

typedef struct{
   double Ia;
   double Ib;
   double Ic;
}Type_infClientSwcApplDigitalTwinMotorPMSM_stOutputs;

typedef struct{
   double Iq;
   double Id;
   double Wm;
   double We;
   double Theta_e;
   double Theta_e_cos;
   double Theta_e_sin;
}Type_SwcApplDigitalTwinMotorPMSM_stIntermediate;

typedef struct{
   double Fqx3;
   double Fdxsqrt3;
   double Vq;
   double Vd;
   double Dq;
   double Dd;
   double Te;
   double DWm;
   double Theta_m; // TBD: Remove
   double Ialpha;
   double Ibeta;
   double Ibetaxsqrt3p0;
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

