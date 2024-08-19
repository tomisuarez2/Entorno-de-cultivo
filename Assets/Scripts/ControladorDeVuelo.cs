using CalculoMatricial; // Clase creada para el calculo matricial.
using System;
using UnityEngine;
using UnityEngine.InputSystem;

public class ControladorDeVuelo : MonoBehaviour
{
    //===========
    // Atributos.
    //===========

    //---------------------------------------------------
    // Habilitación/deshabilitación controlador de vuelo.
    //---------------------------------------------------

    [HideInInspector]
    public bool onOff;

    //---------------------------
    // Mando manual o automático.
    //---------------------------

    [HideInInspector]
    public bool manAut = true; // Si hay señal RC, automáticamente pasa a manual. Del mismo RC pasa a automático.

    //----------------------
    // Computadora de vuelo.
    //----------------------

    public ComputadoraDeVuelo computadora;

    //------------------------
    // Comandos de movimiento.
    //------------------------

    [HideInInspector]
    public float comandoRoll;
    [HideInInspector]
    public float comandoPitch;
    [HideInInspector]
    public float comandoYaw;
    [HideInInspector]
    public float comandoAltitud;

    //-------------------------
    // Input Scaling Constants.
    //-------------------------
   
    private float kPhi = 0.3030f;        // Constante de escalado para comando de roll.
    private float kTheta = 0.3030f;      // Constante de escalado para comando de pitch.
    private float kPsi = 0.0027f;        // Constante de escalado para comando de yaw.
    private float kAltura = 0.0028f;     // Constante de escalado para comando de altitud.

    //------------------------------------
    // Referencias actuales de movimiento.
    //------------------------------------

    [HideInInspector]
    public float rPhi = 0f;        // Referencia actual de roll.
    [HideInInspector]
    public float rTheta = 0f;      // Referencia actual de pitch.
    [HideInInspector]
    public float rPsi = 0f;        // Referencia actual de yaw.
    [HideInInspector]
    public float rAltura = 0f;     // Referencia actual de altitud.

    //---------------------------------------
    // Referencias máximas para Roll y Pitch.
    //---------------------------------------

    [HideInInspector]
    public float refPhiMax = 15f * Mathf.Deg2Rad;
    [HideInInspector]
    public float refThetaMax = 15f * Mathf.Deg2Rad;

    //-----------------------------
    // Referencia de la superficie.
    //-----------------------------

    [HideInInspector]
    public float rSuperficie = 0f;

    //-----------------------------
    // Referencia de altura máxima.
    //-----------------------------

    private float rAlturaMax = 55f;       // Altura máxima de 30 m respecto del suelo establecido.

    //-------------------------------------
    // Constantes de ganancia proporcional.
    //-------------------------------------

    private float kpPhi = 0.1176f;        // Constante de ganacia proporcional en control de roll.
    private float kpTheta = 0.1176f;      // Constante de ganacia proporcional en control de pitch.
    private float kpPsi = 0.1176f;        // Constante de ganacia proporcional en control de yaw.
    private float kpAltura = 12.3973f;    // Constante de ganacia proporcional en control de altitud.

    //---------------------------------
    // Constantes de ganancia integral.
    //---------------------------------

    private float kiPhi = 0.0116f;        // Constante de ganacia integral en control de roll.
    private float kiTheta = 0.0116f;      // Constante de ganacia integral en control de pitch.
    private float kiPsi = 0.0146f;        // Constante de ganacia integral en control de yaw.
    private float kiAltura = 0.0146f;     // Constante de ganacia integral en control de altitud.

    //-----------------------------------
    // Constantes de ganancia derivativa.
    //-----------------------------------

    private float kdPhi = 0.0781f;        // Constante de ganacia derivativa en control de roll.
    private float kdTheta = 0.0781f;      // Constante de ganacia derivativa en control de pitch.
    private float kdPsi = 0.0781f;        // Constante de ganacia derivativa en control de yaw.
    private float kdAltura = 16.4687f;    // Constante de ganacia derivativa en control de altitud.

    //--------------------------------------------------------------------
    // Tolerancia para evitar oscilaciones que son posiblemente numéricas.
    //--------------------------------------------------------------------

    private float tol = 1e-3f;

    //-------------------------------
    // Unidad Inercial de navegacion.
    //-------------------------------

    [HideInInspector]
    public IMU imu; // Unidad de medición inercial.

    //-----------------------
    // Altímetro barométrico.
    //-----------------------

    [HideInInspector]
    public Altimetro altBar;

    //------------------------
    // Conjuntos motor hélice.
    //------------------------

    [HideInInspector]
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

    private float ePhi = 0f;         // Error en el roll.
    private float eTheta = 0f;       // Error en el pitch.
    private float ePsi = 0f;         // Error en el yaw.
    private float eAltura = 0f;      // Error en la altitud.

    //--------------------
    // Errores anteriores.
    //--------------------

    private float eaPhi = 0f;        // Error anterior en roll.
    private float eaTheta = 0f;      // Error anterior en roll.
    private float eaPsi = 0f;        // Error anterior en roll.
    private float eaAltura = 0f;     // Error anterior en roll.

    //----------------------
    // Sumatorias del error.
    //----------------------

    private float esPhi = 0f;        // Sumatoria del error en roll.
    private float esTheta = 0f;      // Sumatoria del error en pitch.
    private float esPsi = 0f;        // Sumatoria del error en yaw.
    private float esAltura = 0f;     // Sumatoria del error en altitud.

    //------------------------------------------
    // Acciones de control para cada movimiento.
    //------------------------------------------

    [HideInInspector]
    public float accionRoll;
    [HideInInspector]
    public float accionPitch;
    [HideInInspector]
    public float accionYaw;
    [HideInInspector]
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

        // El controlador arranca inhabilitado.
        onOff = false;

        // El controlador arranca en manual.
        manAut = true;
    }

    //---------------------------------------------------------
    // Habilitación / Deshabilitación del controlador de vuelo.
    //---------------------------------------------------------
    private void OnControladorOnOff()
    {
        onOff ^= true; // Cada vez que se llama el método, cambiamos el valor del flag.
    }

    //-----------------------------------------------------------------------
    // Habilitación / Deshabilitación del controlador automático de posición.
    //-----------------------------------------------------------------------
    private void OnPosicionAutomatica()
    {
        if (onOff) // Si se encendió el control...
        {
            bool anterior = manAut;
            if (anterior == false) // Si estaba encendido el control automático...
            {
                // Actualizamos referencias para que cuando se encienda el control manual otra vez, se estabilice.
                ResetearReferencias();
            }
            manAut ^= true; // Cada vez que se llama el método, se cambia el flag.
            computadora.automatico = !manAut;
        }
    }

    //--------------------------------------------------------------------
    // Seteo de coordenadas automáticas para volver a la posición de Home.
    //--------------------------------------------------------------------
    /*
    private void OnRetornoHome()
    {
        if(!manAut) // Si nos encontramos en el modo automático.
        {
            // Buscamos el GameObject con el tag especificado como "Home".
            GameObject home = GameObject.FindWithTag("Home");
            // Verifica si se encontró el objeto
            if (home != null)
            {
                // Obtémos las coordenadas (posición) del objeto.
                computadora.rX = home.transform.position.x;
                computadora.rZ = home.transform.position.z;
                computadora.rAlturaComputadora = 28f; // Posicionamos aprox. 2 m sobre plataforma.
            }
        }
    }
    */

    //---------------------------
    // Actualización de comandos.
    //---------------------------

    private void OnComandoAltitudYaw(InputValue comandoEY)
    {
        comandoAltitud = comandoEY.Get<Vector2>().y;
        comandoYaw = comandoEY.Get<Vector2>().x;
    }

    private void OnComandoRollPitch(InputValue comandoRP)
    {
        comandoPitch = -1 * comandoRP.Get<Vector2>().y; // Invierto el comando de Pitch.
        comandoRoll = comandoRP.Get<Vector2>().x;
    }

    //---------------------------------------------
    // Detector de posición de apoyo de referencia.
    //---------------------------------------------
    void OnCollisionEnter(Collision collision)
    {
        // Detectamos el contacto con el objeto.
        if (collision.gameObject.CompareTag("LandingSurface"))
        {
            // Guardamos la altitud mínima permitida.
            rSuperficie = transform.position.y;
        }
    }

    //-------------------------------------------------
    // Actualización de referencias por control remoto.
    //-------------------------------------------------

    public void ActualizarReferenciasRC()
    {
        // Actualización de referencia de altitud.

        if (comandoAltitud > 0)
        {
            rAltura += kAltura * comandoAltitud;
        }
        else if (comandoAltitud < 0)
        {
            rAltura += 0.1f * kAltura * comandoAltitud; // Hacemos esto así para que no parezca que desploma.
        }
        if (rAltura < rSuperficie)
        {
            rAltura = rSuperficie;
        }
        else if (rAltura > rAlturaMax) // Máxima altura permitida.
        {
            rAltura = rAlturaMax;
        }

        // Actualización de referencia de yaw.

        rPsi += kPsi * comandoYaw;
        if (rPsi > Mathf.PI)
        {
            rPsi -= 2 * Mathf.PI;
        }
        else if (rPsi < -1 * Mathf.PI)
        {
            rPsi += 2 * Mathf.PI;
        }

        // Actualización de referencia de pitch.

        if (Mathf.Abs(comandoPitch) > 0) // Si existe un comando de Pitch...
        {
            rTheta += kTheta * comandoPitch;
            if (Mathf.Abs(rTheta) > refThetaMax) // Si la referencia excede en módulo los 45°, saturamos.
            {
                rTheta = Mathf.Sign(rTheta) * refThetaMax;
            }
        }
        else // Caso contrario, llevamos referencia a 0.
        {
            rTheta = 0f;
        }

        // Actualización de referencia de roll.

        if (Mathf.Abs(comandoRoll) > 0) // Si existe un comando de Roll...
        {
            rPhi += kPhi * comandoRoll;
            if (Mathf.Abs(rPhi) > refPhiMax) // Si la referencia excede en módulo los 45°, saturamos.
            {
                rPhi = Mathf.Sign(rPhi) * refPhiMax;
            }
        }
        else // Caso contrario, llevamos referencia a 0.
        {
            rPhi = 0f;
        }
    }

    //-----------------------------------
    // Actualizacion acciones de control.
    //-----------------------------------

    public void AccionControlRoll(float refRoll, float deltaTime)
    {
        // Error.
        ePhi = refRoll - imu.roll;
        ePhi = ePhi > Mathf.PI ? ePhi -= 2 * Mathf.PI : (ePhi < -1*Mathf.PI ? ePhi += 2 * Mathf.PI : ePhi); // Reacondicionamos en funcion de la discontinuidad.

        // Actualizamos la sumatoria de los errores.
        esPhi += ePhi;

        // Calculamos la acción de control.
        accionRoll = kpPhi * ePhi + kiPhi * esPhi * deltaTime + kdPhi * (ePhi - eaPhi) / deltaTime;
        accionRoll = (accionRoll < tol && accionRoll > -tol) ? accionRoll = 0f : accionRoll; // Eliminamos pequeñas oscilaciones innecesarias.

        // Actualizamos el error anterior.
        eaPhi = ePhi;
    }

    public void AccionControlPitch(float refTheta, float deltaTime)
    {
        // Error.
        eTheta = refTheta - imu.pitch;
        eTheta = eTheta > Mathf.PI ? eTheta -= 2 * Mathf.PI : (eTheta < -1 * Mathf.PI ? eTheta += 2 * Mathf.PI : eTheta); // Reacondicionamos en funcion de la discontinuidad.

        // Actualizamos la sumatoria de los errores.
        esTheta += eTheta;

        // Calculamos la acción de control.
        accionPitch = kpTheta * eTheta + kiTheta * esTheta * deltaTime + kdTheta * (eTheta - eaTheta) / deltaTime;
        accionPitch = (accionPitch < tol && accionPitch > -tol) ? accionPitch = 0f : accionPitch; // Eliminamos pequeñas oscilaciones innecesarias.

        // Actualizamos el error anterior.
        eaTheta = eTheta;
    }

    public void AccionControlYaw(float refPsi, float deltaTime) // Seguir revisando acá.
    {
        // Error.
        ePsi = refPsi - imu.yaw;
        ePsi = ePsi > Mathf.PI ? ePsi -= 2 * Mathf.PI : (ePsi < -1 * Mathf.PI ? ePsi += 2 * Mathf.PI : ePsi); // Reacondicionamos en funcion de la discontinuidad.

        // Actualizamos la sumatoria de los errores.
        esPsi += ePsi;

        // Calculamos la acción de control.
        accionYaw = kpPsi * ePsi + kiPsi * esPsi * deltaTime + kdPsi * (ePsi - eaPsi) / deltaTime;
        accionYaw = (accionYaw < tol && accionYaw > -tol) ? accionYaw = 0f : accionYaw; // Eliminamos pequeñas oscilaciones innecesarias.

        // Actualizamos el error anterior.
        eaPsi = ePsi;
    }

    public void AccionControlEmpuje(float refAltura, float deltaTime, float m, float g) 
    {
        // Error.
        eAltura = (refAltura - altBar.altura);

        // Actualizamos la sumatoria de los errores.
        esAltura += eAltura;

        // Calculamos la acción de control.
        accionEmpuje = m * g + kpAltura * eAltura + kiAltura * esAltura * deltaTime + kdAltura * (eAltura - eaAltura) / deltaTime;
        accionEmpuje = accionEmpuje < 0.75f * m * g ? accionEmpuje = 0.75f * m * g : accionEmpuje; // No hay empuje menor que el planteado.
        accionEmpuje /= Mathf.Cos(imu.roll) * Mathf.Cos(imu.pitch); // Referenciamos a sistema B.

        // Saturación. 
       
        if(accionEmpuje > 1.5f * m * g)
        {
            accionEmpuje = 1.5f * m * g; // No empujan mas de eso los motores.
        }
        
        // Actualizamos el error anterior.
        eaAltura = eAltura;
    }

    //---------------------------------------------------
    // Cómputo de las velocidades de giro de los motores.
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

    //---------------------------------------------
    // Métodos auxiliares para mejorar legibilidad.
    //---------------------------------------------

    private void ActualizarSensores(Rigidbody rb, Transform transformacion)
    {
        imu.ActualizacionGiroscopo(rb, transformacion);
        altBar.ActualizarAltimetro(transformacion);
        computadora.gps.ActualizarGPS(rb, transformacion);
    }

    private void ActualizarReferenciasAutomaticas(float deltaTime)
    {
        computadora.ActualizarApsi(imu.yaw);
        computadora.ActualizarReferenciaActitud(refThetaMax, refPhiMax, deltaTime);
    }

    private void ComputarAccionesControl(float alturaRef, float rollRef, float pitchRef, float yawRef, float deltaTime, float m, float g)
    {
        AccionControlEmpuje(alturaRef, deltaTime, m, g);
        AccionControlRoll(rollRef, deltaTime);
        AccionControlPitch(pitchRef, deltaTime);
        AccionControlYaw(yawRef, deltaTime);
    }

    private void ResetearReferencias()
    {
        rAltura = altBar.altura;
        rTheta = 0f;
        rPsi = imu.yaw;
        rPhi = 0f;
        accionEmpuje = 0f;
        accionPitch = 0f;
        accionYaw = 0f;
        accionRoll = 0f;
    }

    //--------------------------------------------------------------------------------
    // Método para computar las acciones del controlador en función de los requisitos.
    //--------------------------------------------------------------------------------

    public void ComputarControlador(Rigidbody rb, Transform transformacion, float deltaTime, float m, float g)
    {
        // Actualización de valores de la IMU, altímetro y GPS.
        ActualizarSensores(rb, transformacion);

        if (onOff) // Si el control esta encendido...
        {
            if (manAut) // Si el mando es manual...
            {
                ActualizarReferenciasRC();
                ComputarAccionesControl(rAltura, rPhi, rTheta, rPsi, deltaTime, m, g);
            }
            else // Si el mando es automático...
            {
                ActualizarReferenciasAutomaticas(deltaTime);

                // Si no se alcanzó aun la rotación, solo giramos.
                if (Mathf.Abs(computadora.rPsiComputadora - imu.yaw) > 0.1f)
                {
                    ComputarAccionesControl(computadora.rAlturaComputadora, 0f, 0f, computadora.rPsiComputadora, deltaTime, m, g);
                }
                else
                {
                    ComputarAccionesControl(computadora.rAlturaComputadora, computadora.rPhiComputadora, computadora.rThetaComputadora, computadora.rPsiComputadora, deltaTime, m, g);
                }
            }

            ComputarMotores(deltaTime);
        }
        else
        {
            // Actualizamos referencias para que cuando se encienda el control otra vez, se estabilice.
            ResetearReferencias();
        }
    }
}
