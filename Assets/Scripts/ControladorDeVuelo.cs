using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using CalculoMatricial; // Clase creada para el calculo matricial.

public class ControladorDeVuelo : MonoBehaviour
{
    //===========
    // Atributos.
    //===========

    //------------------------
    // Comandos de movimiento.
    //------------------------

    public float comandoRoll;
    public float comandoPitch;
    public float comandoYaw;
    public float comandoEmpuje;

    //-------------------------
    // Input Scaling Constants.
    //-------------------------
   
    private float kphi = 0.3030f;        // Constante de escalado para comando de roll.
    private float ktheta = 0.3030f;      // Constante de escalado para comando de pitch.
    private float kpsi = 0.0027f;        // Constante de escalado para comando de yaw.
    private float kaltura = 0.0028f;     // Constante de escalado para comando de altitud.

    //------------------------------------
    // Referencias actuales de movimiento.
    //------------------------------------

    private float rphi = 0f;        // Referencia actual de roll.
    private float rtheta = 0f;      // Referencia actual de pitch.
    private float rpsi = 0f;        // Referencia actual de yaw.
    private float raltura = 0.807f; // Referencia actual de altitud.

    //---------------------------------------
    // Referencias máximas para Roll y Pitch.
    //---------------------------------------

    private float refmaxphi = 15f * Mathf.Deg2Rad;
    private float refmaxtheta = 15f * Mathf.Deg2Rad;

    //-------------------------------------
    // Constantes de ganancia proporcional.
    //-------------------------------------

    private float kpphi = 0.1176f;        // Constante de ganacia proporcional en control de roll.
    private float kptheta = 0.1176f;      // Constante de ganacia proporcional en control de pitch.
    private float kppsi = 0.1176f;       // Constante de ganacia proporcional en control de yaw.
    private float kpaltura = 12.3973f;    // Constante de ganacia proporcional en control de altitud.

    //---------------------------------
    // Constantes de ganancia integral.
    //---------------------------------

    private float kiphi = 0.0116f;        // Constante de ganacia integral en control de roll.
    private float kitheta = 0.0116f;      // Constante de ganacia integral en control de pitch.
    private float kipsi = 0.0146f;        // Constante de ganacia integral en control de yaw.
    private float kialtura = 0.0146f;     // Constante de ganacia integral en control de altitud.

    //-----------------------------------
    // Constantes de ganancia derivativa.
    //-----------------------------------

    private float kdphi = 0.0781f;        // Constante de ganacia derivativa en control de roll.
    private float kdtheta = 0.0781f;      // Constante de ganacia derivativa en control de pitch.
    private float kdpsi = 0.0781f;        // Constante de ganacia derivativa en control de yaw.
    private float kdaltura = 16.4687f;    // Constante de ganacia derivativa en control de altitud.

    //--------------------------------------------------------------------
    // Tolerancia para evitar oscilaciones que son posiblemente numéricas.
    //--------------------------------------------------------------------

    private float tol = 1e-5f;

    //-------------------------------
    // Unidad Inercial de navegacion.
    //-------------------------------

    public IMU imu; // Unidad de medición inercial.

    //-----------------------
    // Altímetro barométrico.
    //-----------------------

    public Altimetro altBar;

    //------------------------
    // Conjuntos motor hélice.
    //------------------------

    public MotorHelice[] motores; // Conjuntos motor-hélice.

    //-------------------------------------------
    // Matriz para el computo de las velocidades.
    //-------------------------------------------

    private float[,] invA;

    //------------------------------------------------
    // Vector del cuadrado de las velocidades de giro.
    //------------------------------------------------

    private float[,] omega2;

    //---------
    // Errores.
    //---------

    private float ephi = 0f;         // Error en el roll.
    private float etheta = 0f;       // Error en el roll.
    private float epsi = 0f;         // Error en el roll.
    private float ealtura = 0f;      // Error en el roll.

    //--------------------
    // Errores anteriores.
    //--------------------

    private float eaphi = 0f;        // Error anterior en roll.
    private float eatheta = 0f;      // Error anterior en roll.
    private float eapsi = 0f;        // Error anterior en roll.
    private float eaaltura = 0f;     // Error anterior en roll.

    //----------------------
    // Sumatorias del error.
    //----------------------

    private float esphi = 0f;        // Sumatoria del error en roll.
    private float estheta = 0f;      // Sumatoria del error en pitch.
    private float espsi = 0f;        // Sumatoria del error en yaw.
    private float esaltura = 0f;     // Sumatoria del error en altitud.

    //------------------------------------------
    // Acciones de control para cada movimiento.
    //------------------------------------------

    public float accionRoll;
    public float accionPitch;
    public float accionYaw;
    public float accionEmpuje;

    private float[,] accionesControl; // Acciones de control colocadas en un vector.

    //=========
    // Métodos.
    //=========

    void Awake()
    {
        invA = new float[6, 4];

        invA[0, 0] = 0.042e4f;
        invA[1, 0] = 0.042e4f;
        invA[2, 0] = 0.042e4f;
        invA[3, 0] = 0.042e4f;
        invA[4, 0] = 0.042e4f;
        invA[5, 0] = 0.042e4f;

        invA[0, 1] = 0.5997e4f;
        invA[1, 1] = 0.2999e4f;
        invA[2, 1] = -0.2999e4f;
        invA[3, 1] = -0.5997e4f;
        invA[4, 1] = -0.2999e4f;
        invA[5, 1] = 0.2999e4f;

        invA[0, 2] = -2.0991e4f;
        invA[1, 2] = 2.0991e4f;
        invA[2, 2] = -2.0991e4f;
        invA[3, 2] = 2.0991e4f;
        invA[4, 2] = -2.0991e4f;
        invA[5, 2] = 2.0991e4f;

        invA[0, 3] = 0e4f;
        invA[1, 3] = -0.5194e4f;
        invA[2, 3] = -0.5194e4f;
        invA[3, 3] = 0e4f;
        invA[4, 3] = 0.5194e4f;
        invA[5, 3] = 0.5194e4f;

        omega2 = new float[6, 1];

        accionesControl = new float[4, 1];

        // Definimos sentido de giro de los motores.
        for(int i = 0; i < 6; i++)
        {
            if (motores[i] != null)
            {
                motores[i].sentido = (int)Mathf.Pow(-1, i + 1);
            }
        }
    }

    //---------------------------
    // Actualización de comandos.
    //---------------------------

    private void OnComandoEmpujeYaw(InputValue comandoEY)
    {
        comandoEmpuje = comandoEY.Get<Vector2>().y;
        comandoYaw = comandoEY.Get<Vector2>().x;
    }

    private void OnComandoRollPitch(InputValue comandoRP)
    {
        comandoPitch = -1 * comandoRP.Get<Vector2>().y; // Invierto el comando de Pitch.
        comandoRoll = comandoRP.Get<Vector2>().x; 
    }

    //-----------------------------------
    // Actualizacion acciones de control.
    //-----------------------------------

    // Antes de llamar a estos métodos se debe realizar una actualización del giroscopio.

    public void AccionControlRoll(float deltaTime)
    {

        // Actualizamos la referencia de roll.
        if (Mathf.Abs(comandoRoll) > 0) // Si existe un comando de Roll...
        {
            rphi += kphi * comandoRoll;
            if (Mathf.Abs(rphi) > refmaxphi) // Si la referencia excede en módulo los 45°, saturamos.
            {
                rphi = Mathf.Sign(rphi) * refmaxphi;
            }
        }
        else // Caso contrario, llevamos referencia a 0.
        {
            rphi = 0f;
        }

        // Error.
        ephi = rphi - imu.roll;
        ephi = ephi > Mathf.PI ? ephi -= 2 * Mathf.PI : (ephi < -1*Mathf.PI ? ephi += 2 * Mathf.PI : ephi); // Reacondicionamos en funcion de la discontinuidad.

        // Actualizamos la sumatoria de los errores.
        esphi += ephi;

        // Calculamos la acción de control.
        accionRoll = kpphi * ephi + kiphi * esphi * deltaTime + kdphi * (ephi - eaphi) / deltaTime;
        accionRoll = (accionRoll < tol && accionRoll > -tol) ? accionRoll = 0f : accionRoll; // Eliminamos pequeñas oscilaciones innecesarias.

        // Actualizamos el error anterior.
        eaphi = ephi;
    }

    public void AccionControlPitch(float deltaTime)
    {

        // Actualizamos la referencia de pitch.
        if(Mathf.Abs(comandoPitch) > 0) // Si existe un comando de Pitch...
        {
            rtheta += ktheta * comandoPitch;
            if(Mathf.Abs(rtheta) > refmaxtheta) // Si la referencia excede en módulo los 45°, saturamos.
            {
                rtheta = Mathf.Sign(rtheta) * refmaxtheta;
            }
        }
        else // Caso contrario, llevamos referencia a 0.
        {
            rtheta = 0f;
        }

        // Error.
        etheta = rtheta - imu.pitch;
        etheta = etheta > Mathf.PI ? etheta -= 2 * Mathf.PI : (etheta < -1 * Mathf.PI ? etheta += 2 * Mathf.PI : etheta); // Reacondicionamos en funcion de la discontinuidad.

        // Actualizamos la sumatoria de los errores.
        estheta += etheta;

        // Calculamos la acción de control.
        accionPitch = kptheta * etheta + kitheta * estheta * deltaTime + kdtheta * (etheta - eatheta) / deltaTime;
        accionPitch = (accionPitch < tol && accionPitch > -tol) ? accionPitch = 0f : accionPitch; // Eliminamos pequeñas oscilaciones innecesarias.

        // Actualizamos el error anterior.
        eatheta = etheta;
    }

    public void AccionControlYaw(float deltaTime) // Seguir revisando acá.
    {

        // Actualizamos la referencia de yaw.
        rpsi += kpsi * comandoYaw;
        if(rpsi > Mathf.PI)
        {
            rpsi -= 2 * Mathf.PI;
        }
        else if(rpsi < -1 * Mathf.PI)
        {
            rpsi += 2 * Mathf.PI;
        }

        // Error.
        epsi = rpsi - imu.yaw;
        epsi = epsi > Mathf.PI ? epsi -= 2 * Mathf.PI : (epsi < -1 * Mathf.PI ? epsi += 2 * Mathf.PI : epsi); // Reacondicionamos en funcion de la discontinuidad.

        // Actualizamos la sumatoria de los errores.
        espsi += epsi;

        // Calculamos la acción de control.
        accionYaw = kppsi * epsi + kipsi * espsi * deltaTime + kdpsi * (epsi - eapsi) / deltaTime;
        accionYaw = (accionYaw < tol && accionYaw > -tol) ? accionYaw = 0f : accionYaw; // Eliminamos pequeñas oscilaciones innecesarias.

        // Actualizamos el error anterior.
        eapsi = epsi;
    }

    public void AccionControlEmpuje(float deltaTime, float m, float g) 
    {

        // Actualizamos la referencia de altitud.
        raltura += kaltura * comandoEmpuje;
        if (raltura < 0.807f)
        {
            raltura = 0.807f;
        }

        // Error.
        ealtura = (raltura - altBar.altura);

        // Actualizamos la sumatoria de los errores.
        esaltura += ealtura;

        // Calculamos la acción de control.
        accionEmpuje = m * g + kpaltura * ealtura + kialtura * esaltura * deltaTime + kdaltura * (ealtura - eaaltura) / deltaTime;
        accionEmpuje = accionEmpuje < 0 ? accionEmpuje = 0 : accionEmpuje; // No hay empuje negativo.
        accionEmpuje /= Mathf.Cos(imu.roll) * Mathf.Cos(imu.pitch); // Referenciamos a sistema B.
        
        // Actualizamos el error anterior.
        eaaltura = ealtura;
    }

    //---------------------------------------------------
    // Computo de las velocidades de giro de los motores.
    //---------------------------------------------------

    public void ComputarMotores(float deltaTime)
    {
        accionesControl[0, 0] = accionEmpuje;
        accionesControl[1, 0] = accionPitch;
        accionesControl[2, 0] = accionYaw;
        accionesControl[3, 0] = accionRoll;

        omega2 = OperacionesMatrices.Producto(invA, accionesControl); // Computamos velocidades.

        for (int i = 0; i < 6; i++)  // Especificamos velocidades y rotamos.
        {
            if (motores[i] != null)
            {
                motores[i].velocidadRotacion = Mathf.Sqrt(omega2[i, 0]); // Velocidades en rad/s.
                motores[i].RotacionConjunto(deltaTime);                  // Rotamos.
            }
        }

    }

    //---------------------------------------------------------
    // Método para computar todas las acciones del controlador.
    //---------------------------------------------------------

    public void ComputarControlador(Rigidbody rb, Transform transformacion, float deltaTime, float m, float g)
    {
        //---------------------------------------------------
        // Actualización de valores de la IMU y el altímetro.
        //---------------------------------------------------

        imu.ActualizacionGiroscopo(rb, transformacion);
        altBar.ActualizarAltimetro(transformacion);

        //-----------------------------------------------------
        // Actualización de las acciones de control requeridas.
        //-----------------------------------------------------

        AccionControlEmpuje(deltaTime, m, g);
        AccionControlRoll(deltaTime);
        AccionControlPitch(deltaTime);
        AccionControlYaw(deltaTime);

        //-------------------------------------------------
        // Computamos velocidad de rotacion de los motores.
        //-------------------------------------------------

        ComputarMotores(deltaTime);
    }




}
