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

class Type_infClientSwcApplDigitalTwinMotorPMSM{
   public:
      const Type_infClientSwcApplDigitalTwinMotorPMSM_stInputs*  pcstInputs;
            Type_infClientSwcApplDigitalTwinMotorPMSM_stOutputs* pstOutputs;
      virtual void InitFunction(
            const Type_infClientSwcApplDigitalTwinMotorPMSM_stInputs*  lpcstInputs
         ,        Type_infClientSwcApplDigitalTwinMotorPMSM_stOutputs* lpstOutputs
      ) = 0;
      virtual void MainFunction(void) = 0;
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

