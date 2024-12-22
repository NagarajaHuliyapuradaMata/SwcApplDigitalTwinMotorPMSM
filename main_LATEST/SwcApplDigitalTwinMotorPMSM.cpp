/******************************************************************************/
/* File   : SwcApplDigitalTwinMotorPMSM.cpp                                   */
/* Author : Nagaraja HULIYAPURADA-MATA                                        */
/* Date   : 01.02.1982                                                        */
/******************************************************************************/

/******************************************************************************/
/* #INCLUDES                                                                  */
/******************************************************************************/
#include "TypesStd.hpp"

#include "infClientSwcApplDigitalTwinMotorPMSM.hpp"

#include "ProjectSystem.hpp"

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
   double Lq;
   double Ld;
   double flux;
   double Rs;
   double J;
   double B;
   uint8  p;
}Type_CfgSwcApplDigitalTwinMotorPMSM_st;

typedef struct{
   double Vq;
   double Vd;
   double Iq;
   double Id;
   double Te;
   double Wm;
   double We;
   double Theta_e;
   double Theta_m;
}Type_SwcApplDigitalTwinMotorPMSM_stIntermediate;

class Type_SwcApplDigitalTwinMotorPMSM : public Type_infClientSwcApplDigitalTwinMotorPMSM{
   private:
      const Type_CfgSwcApplDigitalTwinMotorPMSM_st*         pcstCfgst;
            Type_SwcApplDigitalTwinMotorPMSM_stIntermediate stIntermediate;

   public:
      void InitFunction(
            const Type_infClientSwcApplDigitalTwinMotorPMSM_stInputs*  lpcstInputs
         ,        Type_infClientSwcApplDigitalTwinMotorPMSM_stOutputs* lpstOutputs
      );
      void MainFunction(void);
};

/******************************************************************************/
/* CONSTS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* PARAMS                                                                     */
/******************************************************************************/
const Type_CfgSwcApplDigitalTwinMotorPMSM_st CfgSwcApplDigitalTwinMotorPMSM_st = {
      0.00334   /* H        */
   ,  0.00333   /* H        */
   ,  0.171     /* Wb       */
   ,  0.4578    /* ohm      */
   ,  0.001469  /* Kg m2    */
   ,  0.0003035 /* unitless */
   ,  8
};

/******************************************************************************/
/* OBJECTS                                                                    */
/******************************************************************************/
Type_SwcApplDigitalTwinMotorPMSM           SwcApplDigitalTwinMotorPMSM;
Type_infClientSwcApplDigitalTwinMotorPMSM* pinfClientSwcApplDigitalTwinMotorPMSM = &SwcApplDigitalTwinMotorPMSM;

/******************************************************************************/
/* FUNCTIONS                                                                  */
/******************************************************************************/
void Type_SwcApplDigitalTwinMotorPMSM::InitFunction(
      const Type_infClientSwcApplDigitalTwinMotorPMSM_stInputs*  lpcstInputs
   ,        Type_infClientSwcApplDigitalTwinMotorPMSM_stOutputs* lpstOutputs
){
   this->pcstInputs             = lpcstInputs;
   this->pstOutputs             = lpstOutputs;
   this->pcstCfgst              = &CfgSwcApplDigitalTwinMotorPMSM_st;
   this->stIntermediate.Vq      = 0.0;
   this->stIntermediate.Vd      = 0.0;
   this->stIntermediate.Iq      = 0.0;
   this->stIntermediate.Id      = 0.0;
   this->stIntermediate.Te      = 0.0;
   this->stIntermediate.Wm      = 0.0;
   this->stIntermediate.We      = 0.0;
   this->stIntermediate.Theta_e = 0.0;
   this->stIntermediate.Theta_m = 0.0;
}

#include <cmath>
void Type_SwcApplDigitalTwinMotorPMSM::MainFunction(void){
   double Iq_z        = this->stIntermediate.Iq;
   double Id_z        = this->stIntermediate.Id;
   double f64Fqx3     = 2*this->pcstInputs->Va - this->pcstInputs->Vb - this->pcstInputs->Vc;
   double f64Fdxsqrt3 =                        - this->pcstInputs->Vb + this->pcstInputs->Vc;
   this->stIntermediate.Vq = f64Fqx3*cos(this->stIntermediate.Theta_e)/3.0 - f64Fdxsqrt3*sin(this->stIntermediate.Theta_e)/sqrt(3.0);
   this->stIntermediate.Vd = f64Fqx3*sin(this->stIntermediate.Theta_e)/3.0 + f64Fdxsqrt3*cos(this->stIntermediate.Theta_e)/sqrt(3.0);

   double lf64Dq = (
         this->stIntermediate.Vq
      -  this->pcstCfgst->Ld   * (Id_z * this->stIntermediate.We)
      -  this->pcstCfgst->Rs   * Iq_z
      -  this->pcstCfgst->flux * this->stIntermediate.We
   )/this->pcstCfgst->Lq;
   this->stIntermediate.Iq += lf64Dq * dt;
   double lf64Dd = (
         this->stIntermediate.Vd
      +  this->pcstCfgst->Lq * (Iq_z * this->stIntermediate.We)
      -  this->pcstCfgst->Rs * Id_z
   )/this->pcstCfgst->Ld;
   this->stIntermediate.Id += lf64Dd * dt;

   this->stIntermediate.Te = 3 * this->pcstCfgst->p * this->stIntermediate.Iq * (
         this->pcstCfgst->flux
      +  (this->pcstCfgst->Ld - this->pcstCfgst->Lq) * this->stIntermediate.Id
   ) / 4;

   double lf64DWm = (
                        +  this->stIntermediate.Te
                        -  this->pcstInputs->Tm
                        -  this->pcstCfgst->B * this->stIntermediate.Wm
                    ) / this->pcstCfgst->J;
   this->stIntermediate.Wm      += lf64DWm * dt;
   this->stIntermediate.We       = this->pcstCfgst->p * this->stIntermediate.Wm / 2;
   this->stIntermediate.Theta_e += this->stIntermediate.We * dt;
   this->stIntermediate.Theta_m  = 2 * this->stIntermediate.Theta_e / this->pcstCfgst->p;

   double lf64Ialpha =  this->stIntermediate.Iq * cos(this->stIntermediate.Theta_e) + this->stIntermediate.Id * sin(this->stIntermediate.Theta_e);
   double lf64Ibeta  = -this->stIntermediate.Iq * sin(this->stIntermediate.Theta_e) + this->stIntermediate.Id * cos(this->stIntermediate.Theta_e);
   this->pstOutputs->Ia =   lf64Ialpha;
   this->pstOutputs->Ib = -(lf64Ialpha + sqrt(3)*lf64Ibeta)/2;
   this->pstOutputs->Ic = -(lf64Ialpha - sqrt(3)*lf64Ibeta)/2;
}

/******************************************************************************/
/* EOF                                                                        */
/******************************************************************************/

