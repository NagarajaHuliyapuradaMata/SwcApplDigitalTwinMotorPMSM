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
#define double_Lq   0.00334   /* H        */
#define double_Ld   0.00333   /* H        */
#define double_flux 0.171     /* Wb       */
#define double_Rs   0.4578    /* ohm      */
#define double_J    0.001469  /* Kg m2    */
#define double_B    0.0003035 /* unitless */
#define double_p    8.0       /* poles    */

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
   double p;
}Type_CfgSwcApplDigitalTwinMotorPMSM_st;

class Type_SwcApplDigitalTwinMotorPMSM : public Type_infClientSwcApplDigitalTwinMotorPMSM{
   private:
      const Type_CfgSwcApplDigitalTwinMotorPMSM_st*              pcstCfgst;
      const Type_infClientSwcApplDigitalTwinMotorPMSM_stInputs*  pcstInputs;
            Type_SwcApplDigitalTwinMotorPMSM_stIntermediate*     pstIntermediate;
            Type_infClientSwcApplDigitalTwinMotorPMSM_stOutputs* pstOutputs;

   public:
      void InitFunction(
            const Type_infClientSwcApplDigitalTwinMotorPMSM_stInputs*  lpcstInputs
         ,        Type_SwcApplDigitalTwinMotorPMSM_stIntermediate*     lpstIntermediate
         ,        Type_infClientSwcApplDigitalTwinMotorPMSM_stOutputs* lpstOutputs
      );
      void MainFunction   (void);
      void DeInitFunction (void);
};

/******************************************************************************/
/* CONSTS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* PARAMS                                                                     */
/******************************************************************************/
const Type_CfgSwcApplDigitalTwinMotorPMSM_st CfgSwcApplDigitalTwinMotorPMSM_st = {
      double_Lq
   ,  double_Ld
   ,  double_flux
   ,  double_Rs
   ,  double_J
   ,  double_B
   ,  double_p
};

/******************************************************************************/
/* OBJECTS                                                                    */
/******************************************************************************/
Type_SwcApplDigitalTwinMotorPMSM           SwcApplDigitalTwinMotorPMSM;
Type_infClientSwcApplDigitalTwinMotorPMSM* pinfClientSwcApplDigitalTwinMotorPMSM = &SwcApplDigitalTwinMotorPMSM;

/******************************************************************************/
/* FUNCTIONS                                                                  */
/******************************************************************************/
#include "interface_LogAndTrace.hpp"
void Type_SwcApplDigitalTwinMotorPMSM::InitFunction(
      const Type_infClientSwcApplDigitalTwinMotorPMSM_stInputs*  lpcstInputs
   ,        Type_SwcApplDigitalTwinMotorPMSM_stIntermediate*     lpstIntermediate
   ,        Type_infClientSwcApplDigitalTwinMotorPMSM_stOutputs* lpstOutputs
){
   pinfLog->InitFunction("log_pProjectVirtualEcu_DigitalTwinMotorPMSM_Rb_calcFloatingPoint.csv"); // TBD: Remove hardcode and pass control promt argument
   this->pcstInputs                   = lpcstInputs;
   this->pstIntermediate              = lpstIntermediate;
   this->pstOutputs                   = lpstOutputs;
   this->pcstCfgst                    = &CfgSwcApplDigitalTwinMotorPMSM_st;
   this->pstIntermediate->Iq          = 0.0;
   this->pstIntermediate->Id          = 0.0;
   this->pstIntermediate->Wm          = 0.0;
   this->pstIntermediate->We          = 0.0;
   this->pstIntermediate->Theta_e     = 0.0;
   this->pstIntermediate->Theta_e_cos = 1.0;
   this->pstIntermediate->Theta_e_sin = 0.0;
}

#include <cmath>
const double dt = double_dt;
void Type_SwcApplDigitalTwinMotorPMSM::MainFunction(void){
   Type_SwcApplDigitalTwinMotorPMSM_stLocals stLocals;
   stLocals.Fqx3     = 2*this->pcstInputs->Va - this->pcstInputs->Vb - this->pcstInputs->Vc;
   stLocals.Fdxsqrt3 =                        - this->pcstInputs->Vb + this->pcstInputs->Vc;

   stLocals.Vq = (
         (
               (
                     stLocals.Fqx3
                  *  this->pstIntermediate->Theta_e_cos
               )
            /  3.0
         )
      -  (
               (
                     stLocals.Fdxsqrt3
                  *  this->pstIntermediate->Theta_e_sin
               )
            /  sqrt(3.0)
         )
   );

   stLocals.Vd = (
         (
               (
                     stLocals.Fqx3
                  *  this->pstIntermediate->Theta_e_sin
               )
            /  3.0
         )
      +  (
               (
                     stLocals.Fdxsqrt3
                  *  this->pstIntermediate->Theta_e_cos
               )
            /  sqrt(3.0)
         )
   );

   stLocals.Dq = (
         (
               stLocals.Vq
            -  (
                     this->pcstCfgst->Ld
                  *  (
                           this->pstIntermediate->Id
                        *  this->pstIntermediate->We
                     )
               )
            -  (
                     this->pcstCfgst->Rs
                  *  this->pstIntermediate->Iq
               )
            -  (
                     this->pcstCfgst->flux
                  *  this->pstIntermediate->We
               )
         )
      /  this->pcstCfgst->Lq
   );

   stLocals.Dd = (
         (
               stLocals.Vd
            +  (
                     this->pcstCfgst->Lq
                  *  (
                           this->pstIntermediate->Iq
                        *  this->pstIntermediate->We
                     )
               )
            -  (
                     this->pcstCfgst->Rs
                  *  this->pstIntermediate->Id
               )
         )
      /  this->pcstCfgst->Ld
   );

   this->pstIntermediate->Iq += (
         stLocals.Dq
      *  dt
   );

   this->pstIntermediate->Id += (
         stLocals.Dd
      *  dt
   );

   stLocals.Te = (
         (
               3
            *  (
                     this->pcstCfgst->p
                  *  (
                           this->pstIntermediate->Iq
                        *  (
                                 this->pcstCfgst->flux
                              +  (
                                       (this->pcstCfgst->Ld - this->pcstCfgst->Lq)
                                    *  this->pstIntermediate->Id
                                 )
                           )
                     )
               )
         )
      /  4
   );

   stLocals.DWm = (
         (
            +  stLocals.Te
            -  this->pcstInputs->Tm
            -  (
                     this->pcstCfgst->B
                  *  this->pstIntermediate->Wm
               )
         )
      /  this->pcstCfgst->J
   );

   this->pstIntermediate->Wm += (
         stLocals.DWm
      *  dt
   );

   this->pstIntermediate->We = (
         (
               this->pcstCfgst->p
            *  this->pstIntermediate->Wm
         )
      /  2
   );

   this->pstIntermediate->Theta_e += (
         this->pstIntermediate->We
      *  dt
   );

   this->pstIntermediate->Theta_e_cos = cos(this->pstIntermediate->Theta_e);
   this->pstIntermediate->Theta_e_sin = sin(this->pstIntermediate->Theta_e);

   stLocals.Theta_m = (
         (
               2
            *  this->pstIntermediate->Theta_e
         )
      /  this->pcstCfgst->p
   );

   stLocals.Ialpha = (
         (
               this->pstIntermediate->Iq
            *  this->pstIntermediate->Theta_e_cos
         )
      +  (
               this->pstIntermediate->Id
            *  this->pstIntermediate->Theta_e_sin
         )
   );

   stLocals.Ibeta = (
      -  (
               this->pstIntermediate->Iq
            *  this->pstIntermediate->Theta_e_sin
         )
      +  (
               this->pstIntermediate->Id
            *  this->pstIntermediate->Theta_e_cos
         )
   );

   stLocals.Ibetaxsqrt3p0 = (
         sqrt(3)
      *  stLocals.Ibeta
   );

   this->pstOutputs->Ia = stLocals.Ialpha;

   this->pstOutputs->Ib = -(
         (
               stLocals.Ialpha
            +  stLocals.Ibetaxsqrt3p0
         )
      /  2
   );

   this->pstOutputs->Ic = -(
         (
               stLocals.Ialpha
            -  stLocals.Ibetaxsqrt3p0
         )
      /  2
   );

   pinfLog->MainFunction(
         this->pcstInputs->Tm
      ,  this->pcstInputs->Va
      ,  this->pcstInputs->Vb
      ,  this->pcstInputs->Vc
      ,  stLocals.Fqx3
      ,  stLocals.Fdxsqrt3
      ,  stLocals.Vq
      ,  stLocals.Vd
      ,  stLocals.Dq
      ,  stLocals.Dd
      ,  this->pstIntermediate->Iq
      ,  this->pstIntermediate->Id
      ,  stLocals.Te
      ,  stLocals.DWm
      ,  this->pstIntermediate->Wm
      ,  this->pstIntermediate->We
      ,  this->pstIntermediate->Theta_e
      ,  this->pstIntermediate->Theta_e_cos
      ,  this->pstIntermediate->Theta_e_sin
      ,  stLocals.Theta_m
      ,  stLocals.Ialpha
      ,  stLocals.Ibeta
      ,  this->pstOutputs->Ia
      ,  this->pstOutputs->Ib
      ,  this->pstOutputs->Ic
   );
}

void Type_SwcApplDigitalTwinMotorPMSM::DeInitFunction(void){
   pinfLog->DeInitFunction();
}

/******************************************************************************/
/* EOF                                                                        */
/******************************************************************************/

