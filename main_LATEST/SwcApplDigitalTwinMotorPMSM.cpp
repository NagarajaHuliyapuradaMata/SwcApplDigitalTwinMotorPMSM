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
   s42p21 Lq;
   s42p21 Ld;
   s42p21 flux;
   s42p21 Rs;
   s42p21 J;
   s42p21 B;
   s42p21 p;
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
      double_to_fixed(double_Lq,   scale_s42p21)
   ,  double_to_fixed(double_Ld,   scale_s42p21)
   ,  double_to_fixed(double_flux, scale_s42p21)
   ,  double_to_fixed(double_Rs,   scale_s42p21)
   ,  double_to_fixed(double_J,    scale_s42p21)
   ,  double_to_fixed(double_B,    scale_s42p21)
   ,  double_to_fixed(double_p,    scale_s42p21)
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
   pinfLog->InitFunction("log_pProjectVirtualEcu_DigitalTwinMotorPMSM_Rb_calcFixedPoint.csv"); // TBD: Remove hardcode and pass control promt argument
   this->pcstInputs                   = lpcstInputs;
   this->pstIntermediate              = lpstIntermediate;
   this->pstOutputs                   = lpstOutputs;
   this->pcstCfgst                    = &CfgSwcApplDigitalTwinMotorPMSM_st;
   this->pstIntermediate->Iq          = double_to_fixed(0.0, scale_s42p21);
   this->pstIntermediate->Id          = double_to_fixed(0.0, scale_s42p21);
   this->pstIntermediate->Wm          = double_to_fixed(0.0, scale_s42p21);
   this->pstIntermediate->We          = double_to_fixed(0.0, scale_s42p21);
   this->pstIntermediate->Theta_e     = double_to_fixed(0.0, scale_s42p21);
   this->pstIntermediate->Theta_e_cos = double_to_fixed(1.0, scale_s42p21);
   this->pstIntermediate->Theta_e_sin = double_to_fixed(0.0, scale_s42p21);
}

#include <cmath>
const s42p21        dt      = double_to_fixed(double_dt, scale_s42p21);
const s42p21 s42p21_2p0     = double_to_fixed(2.0,       scale_s42p21);
const s42p21 s42p21_3p0     = double_to_fixed(3.0,       scale_s42p21);
const s42p21 s42p21_4p0     = double_to_fixed(4.0,       scale_s42p21);
const s42p21 s42p21_sqrt3p0 = double_to_fixed(sqrt(3.0), scale_s42p21);

s42p21 s42p21_cos(s42p21 s42p21_Theta){
   return double_to_fixed(
         cos(
            fixed_to_double(
                  s42p21_Theta
               ,  scale_s42p21
            )
         )
      ,  scale_s42p21
   );
}

s42p21 s42p21_sin(s42p21 s42p21_Theta){
   return double_to_fixed(
         sin(
            fixed_to_double(
                  s42p21_Theta
               ,  scale_s42p21
            )
         )
      ,  scale_s42p21
   );
}

void Type_SwcApplDigitalTwinMotorPMSM::MainFunction(void){
   Type_SwcApplDigitalTwinMotorPMSM_stLocals stLocals;
   stLocals.Fqx3     = 2*this->pcstInputs->Va - this->pcstInputs->Vb - this->pcstInputs->Vc;
   stLocals.Fdxsqrt3 =                        - this->pcstInputs->Vb + this->pcstInputs->Vc;

   stLocals.Vq = (
         fixed_div_2(
               fixed_mul_2(
                     stLocals.Fqx3
                  ,  this->pstIntermediate->Theta_e_cos
                  ,  0
                  ,  0
                  ,  scale_s42p21
               )
            ,  s42p21_3p0
            ,  scale_s42p21
            ,  scale_s42p21
         )
      -  fixed_div_2(
               fixed_mul_2(
                     stLocals.Fdxsqrt3
                  ,  this->pstIntermediate->Theta_e_sin
                  ,  0
                  ,  0
                  ,  scale_s42p21
               )
            ,  s42p21_sqrt3p0
            ,  scale_s42p21
            ,  scale_s42p21
         )
   );

   stLocals.Vd = (
         fixed_div_2(
               fixed_mul_2(
                     stLocals.Fqx3
                  ,  this->pstIntermediate->Theta_e_sin
                  ,  0
                  ,  0
                  ,  scale_s42p21
               )
            ,  s42p21_3p0
            ,  scale_s42p21
            ,  scale_s42p21
         )
      +  fixed_div_2(
               fixed_mul_2(
                     stLocals.Fdxsqrt3
                  ,  this->pstIntermediate->Theta_e_cos
                  ,  0
                  ,  0
                  ,  scale_s42p21
               )
            ,  s42p21_sqrt3p0
            ,  scale_s42p21
            ,  scale_s42p21
         )
   );

   stLocals.Dq = fixed_div_2(
         (
               stLocals.Vq
            -  fixed_mul_2(
                     this->pcstCfgst->Ld
                  ,  fixed_mul_2(
                           this->pstIntermediate->Id
                        ,  this->pstIntermediate->We
                        ,  0
                        ,  0
                        ,  scale_s42p21
                     )
                  ,  0
                  ,  0
                  ,  scale_s42p21
               )
            -  fixed_mul_2(
                     this->pcstCfgst->Rs
                  ,  this->pstIntermediate->Iq
                  ,  0
                  ,  0
                  ,  scale_s42p21
               )
            -  fixed_mul_2(
                     this->pcstCfgst->flux
                  ,  this->pstIntermediate->We
                  ,  0
                  ,  0
                  ,  scale_s42p21
               )
         )
      ,  this->pcstCfgst->Lq
      ,  scale_s42p21
      ,  scale_s42p21
   );

   stLocals.Dd = fixed_div_2(
         (
               stLocals.Vd
            +  fixed_mul_2(
                     this->pcstCfgst->Lq
                  ,  fixed_mul_2(
                           this->pstIntermediate->Iq
                        ,  this->pstIntermediate->We
                        ,  0
                        ,  0
                        ,  scale_s42p21
                     )
                  ,  0
                  ,  0
                  ,  scale_s42p21
               )
            -  fixed_mul_2(
                     this->pcstCfgst->Rs
                  ,  this->pstIntermediate->Id
                  ,  0
                  ,  0
                  ,  scale_s42p21
               )
         )
      ,  this->pcstCfgst->Ld
      ,  scale_s42p21
      ,  scale_s42p21
   );

   this->pstIntermediate->Iq += fixed_mul_2(
         stLocals.Dq
      ,  dt
      ,  0
      ,  0
      ,  scale_s42p21
   );

   this->pstIntermediate->Id += fixed_mul_2(
         stLocals.Dd
      ,  dt
      ,  0
      ,  0
      ,  scale_s42p21
   );

   stLocals.Te = fixed_div_2(
         fixed_mul_2(
               s42p21_3p0
            ,  fixed_mul_2(
                     this->pcstCfgst->p
                  ,  fixed_mul_2(
                           this->pstIntermediate->Iq
                        ,  (
                                 this->pcstCfgst->flux
                              +  fixed_mul_2(
                                       (this->pcstCfgst->Ld - this->pcstCfgst->Lq)
                                    ,  this->pstIntermediate->Id
                                    ,  0
                                    ,  0
                                    ,  scale_s42p21
                                 )
                           )
                        ,  0
                        ,  0
                        ,  scale_s42p21
                     )
                  ,  0
                  ,  0
                  ,  scale_s42p21
               )
            ,  0
            ,  0
            ,  scale_s42p21
         )
      ,  s42p21_4p0
      ,  scale_s42p21
      ,  scale_s42p21
   );

   stLocals.DWm = fixed_div_2(
         (
            +  stLocals.Te
            -  this->pcstInputs->Tm
            -  fixed_mul_2(
                     this->pcstCfgst->B
                  ,  this->pstIntermediate->Wm
                  ,  0
                  ,  0
                  ,  scale_s42p21
               )
         )
      ,  this->pcstCfgst->J
      ,  scale_s42p21
      ,  scale_s42p21
   );

   this->pstIntermediate->Wm += fixed_mul_2(
         stLocals.DWm
      ,  dt
      ,  0
      ,  0
      ,  scale_s42p21
   );

   this->pstIntermediate->We = fixed_div_2(
         fixed_mul_2(
               this->pcstCfgst->p
            ,  this->pstIntermediate->Wm
            ,  0
            ,  0
            ,  scale_s42p21
         )
      ,  s42p21_2p0
      ,  scale_s42p21
      ,  scale_s42p21
   );

   this->pstIntermediate->Theta_e += fixed_mul_2(
         this->pstIntermediate->We
      ,  dt
      ,  0
      ,  0
      ,  scale_s42p21
   );

   this->pstIntermediate->Theta_e_cos = s42p21_cos(this->pstIntermediate->Theta_e);
   this->pstIntermediate->Theta_e_sin = s42p21_sin(this->pstIntermediate->Theta_e);

   stLocals.Theta_m = fixed_div_2(
         fixed_mul_2(
               s42p21_2p0
            ,  this->pstIntermediate->Theta_e
            ,  0
            ,  0
            ,  scale_s42p21
         )
      ,  this->pcstCfgst->p
      ,  scale_s42p21
      ,  scale_s42p21
   );

   stLocals.Ialpha = (
         fixed_mul_2(
               this->pstIntermediate->Iq
            ,  this->pstIntermediate->Theta_e_cos
            ,  0
            ,  0
            ,  scale_s42p21
         )
      +  fixed_mul_2(
               this->pstIntermediate->Id
            ,  this->pstIntermediate->Theta_e_sin
            ,  0
            ,  0
            ,  scale_s42p21
         )
   );

   stLocals.Ibeta = (
      -  fixed_mul_2(
               this->pstIntermediate->Iq
            ,  this->pstIntermediate->Theta_e_sin
            ,  0
            ,  0
            ,  scale_s42p21
         )
      +  fixed_mul_2(
               this->pstIntermediate->Id
            ,  this->pstIntermediate->Theta_e_cos
            ,  0
            ,  0
            ,  scale_s42p21
         )
   );

   stLocals.Ibetaxsqrt3p0 =  fixed_mul_2(
         s42p21_sqrt3p0
      ,  stLocals.Ibeta
      ,  0
      ,  0
      ,  scale_s42p21
   );

   this->pstOutputs->Ia = stLocals.Ialpha;

   this->pstOutputs->Ib = -fixed_div_2(
         (
               stLocals.Ialpha
            +  stLocals.Ibetaxsqrt3p0
         )
      ,  s42p21_2p0
      ,  scale_s42p21
      ,  scale_s42p21
   );

   this->pstOutputs->Ic = -fixed_div_2(
         (
               stLocals.Ialpha
            -  stLocals.Ibetaxsqrt3p0
         )
      ,  s42p21_2p0
      ,  scale_s42p21
      ,  scale_s42p21
   );

   pinfLog->MainFunction(
         fixed_to_double(this->pcstInputs->Tm,               scale_s42p21)
      ,  fixed_to_double(this->pcstInputs->Va,               scale_s42p21)
      ,  fixed_to_double(this->pcstInputs->Vb,               scale_s42p21)
      ,  fixed_to_double(this->pcstInputs->Vc,               scale_s42p21)
      ,  fixed_to_double(stLocals.Fqx3,                      scale_s42p21)
      ,  fixed_to_double(stLocals.Fdxsqrt3,                  scale_s42p21)
      ,  fixed_to_double(stLocals.Vq,                        scale_s42p21)
      ,  fixed_to_double(stLocals.Vd,                        scale_s42p21)
      ,  fixed_to_double(stLocals.Dq,                        scale_s42p21)
      ,  fixed_to_double(stLocals.Dd,                        scale_s42p21)
      ,  fixed_to_double(this->pstIntermediate->Iq,          scale_s42p21)
      ,  fixed_to_double(this->pstIntermediate->Id,          scale_s42p21)
      ,  fixed_to_double(stLocals.Te,                        scale_s42p21)
      ,  fixed_to_double(stLocals.DWm,                       scale_s42p21)
      ,  fixed_to_double(this->pstIntermediate->Wm,          scale_s42p21)
      ,  fixed_to_double(this->pstIntermediate->We,          scale_s42p21)
      ,  fixed_to_double(this->pstIntermediate->Theta_e,     scale_s42p21)
      ,  fixed_to_double(this->pstIntermediate->Theta_e_cos, scale_s42p21)
      ,  fixed_to_double(this->pstIntermediate->Theta_e_sin, scale_s42p21)
      ,  fixed_to_double(stLocals.Theta_m,                   scale_s42p21)
      ,  fixed_to_double(stLocals.Ialpha,                    scale_s42p21)
      ,  fixed_to_double(stLocals.Ibeta,                     scale_s42p21)
      ,  fixed_to_double(this->pstOutputs->Ia,               scale_s42p21)
      ,  fixed_to_double(this->pstOutputs->Ib,               scale_s42p21)
      ,  fixed_to_double(this->pstOutputs->Ic,               scale_s42p21)
   );
}

void Type_SwcApplDigitalTwinMotorPMSM::DeInitFunction(void){
   pinfLog->DeInitFunction();
}

/******************************************************************************/
/* EOF                                                                        */
/******************************************************************************/

