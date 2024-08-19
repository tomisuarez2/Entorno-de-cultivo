using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngineInternal;

//==========================
// Clase ComputadoraDeVuelo.
//==========================

/*
 Esta clase se encarga de comandar el movimiento autonomo del Dron. Da referencias de altitud y
 actitud a la clase ControladorDeVuelo.
 */

public class ComputadoraDeVuelo : MonoBehaviour
{
    //===========
    // Atributos.
    //===========

    //---------------------------------------
    // Flag de detección de vuelo automático.
    //---------------------------------------

    public bool automatico = false;

    //-----
    // GPS.
    //-----

    public GPS gps;

    //-----------------------------------------------------
    // Constantes para el control de posición proporcional.
    //-----------------------------------------------------

    private float kpX = 0.3f;
    private float kpZ = 0.3f;

    //---------------------------------------------------
    // Constantes para el control de posición derivativo.
    //---------------------------------------------------

    private float kdX = 0.1f;
    private float kdZ = 0.1f;

    //------------------------------------------------
    // Constantes para el control de posición integral.
    //------------------------------------------------

    private float kiX = 0f;  // El control integral arruina todo esto. No usar. Entender por qué.
    private float kiZ = 0f;

    //--------------------------------------------------------------------
    // Tolerancia para evitar oscilaciones que son posiblemente numéricas.
    //--------------------------------------------------------------------

    private float tol = 1e-5f;

    //-------------------------------
    // Matriz de transformación Apsi.
    //-------------------------------

    private float[,] Apsi;

    //-----------------------------------
    // Referencia de posición horizontal.
    //-----------------------------------

    public float rX = 0; // Referencia de posición en X.
    public float rZ = 0; // Referencia de posición en Z.
    private float raX; // Referencia anterior de X.
    private float raZ; // Referencia anterior de Z.

    //---------------------
    // Errores de posición.
    //---------------------

    private float eX;
    private float eZ;

    //--------------------------------
    // Errores anteriores de posición.
    //--------------------------------

    private float eaX;
    private float eaZ;

    //----------------------
    // Sumatorias del error.
    //----------------------

    private float esX;
    private float esZ;

    //----------------------------------------------------
    // Relacion entre referencias para mantener dirección.
    //----------------------------------------------------

    private float thetaRoll; // Esto implica ref. de pitch sobre ref. de roll.

    //-------------------------------------------------
    // Flags que indican saturación en las referencias.
    //-------------------------------------------------

    private bool satRoll;
    private bool satTheta;

    //---------------------------------------------------------------------------------------------
    // Referencia de roll y pitch calculada por la computadora para control de posición horizontal.
    //---------------------------------------------------------------------------------------------

    [HideInInspector]
    public float rPhiComputadora;
    [HideInInspector]
    public float rThetaComputadora;

    //------------------------------------------------------------
    // Referencia de yaw consecuente con la referencia horizontal.
    //------------------------------------------------------------

    [HideInInspector]
    public float rPsiComputadora;

    //--------------------------------
    // Referencia de altura requerida.
    //--------------------------------

    public float rAlturaComputadora;

    //---------------------------
    // Posición y altura de Home.
    //---------------------------

    private float rXHome;
    private float rZHome;
    private float rAlturaHome;

    //--------------------
    // Clase pulverizador.
    //--------------------

    public Pulverizador pulverizador;

    //=========
    // Métodos.
    //=========

    private void Awake()
    {
        Apsi = new float[2, 2];

        // Calculamos la posición de "Home".

        // Buscamos el GameObject con el tag especificado como "Home".
        GameObject home = GameObject.FindWithTag("Home");
        // Verifica si se encontró el objeto
        if (home != null)
        {
            // Obtémos las coordenadas (posición) del objeto.
            rXHome = home.transform.position.x;
            rZHome = home.transform.position.z;
            rAlturaHome = 28f; // Posicionamos aprox. 2 m sobre plataforma.
        }

        // Calculamos la primera vez rPsiComputadora.

        rPsiComputadora = Mathf.Atan2(-rX + gps.posicionX, -rZ + gps.posicionZ);
    }

    //-----------------
    // Actualizar Apsi.
    //-----------------

    public void ActualizarApsi(float yaw)
    {
        Apsi[0, 0] = Mathf.Sin(yaw);
        Apsi[0, 1] = -1 * Mathf.Cos(yaw);
        Apsi[1, 0] = Mathf.Cos(yaw);
        Apsi[1, 1] = Mathf.Sin(yaw);
    }

    //--------------------------------
    // Control de posición horizontal.
    //--------------------------------
    public void ActualizarReferenciaActitud(float rPitchMax, float rRollMax, float deltaTime)
    {
        // Error de posición.
        eX = rX - gps.posicionX;
        eZ = rZ - gps.posicionZ;

        // Actualización rPsiComputadora.
        if ((rX != raX) | (rZ != raZ)) // Si hay cambio de referencia...
        {
            rPsiComputadora = Mathf.Atan2(-eX, -eZ);
        }

        // Actualizamos la sumatoria de los errores.
        esX += eX;
        esZ += eZ;

        // Actualización de referencia.
        rThetaComputadora = (1 / 9.81f) * (Apsi[0, 0] * (kpX * eX + kdX * (eX - eaX) / deltaTime + kiX * esX * deltaTime) + Apsi[1, 0] * (kpZ * eZ + kdZ * (eZ - eaZ) / deltaTime + kiZ * esZ * deltaTime));
        rPhiComputadora = (1 / 9.81f) * (Apsi[0, 1] * (kpX * eX + kdX * (eX - eaX) / deltaTime + kiX * esX * deltaTime) + Apsi[1, 1] * (kpZ * eZ + kdZ * (eZ - eaZ) / deltaTime + kiZ * esZ * deltaTime));

        // Saturamos con dirección garantizada.
        thetaRoll = rThetaComputadora / rPhiComputadora;

        if (Mathf.Abs(rThetaComputadora) > rPitchMax)
        {
            rThetaComputadora = Mathf.Sign(rThetaComputadora) * rPitchMax;
            satTheta = true;
        }
        else
        {
            satTheta = false;
        }
        if (Mathf.Abs(rPhiComputadora) > rRollMax)
        {
            rPhiComputadora = Mathf.Sign(rPhiComputadora) * rRollMax;
            satRoll = true;
        }
        else
        {
            satRoll = false;
        }

        if((satTheta & !satRoll) | (satTheta & satRoll & (thetaRoll > 1)))
        {
            /*
             * Si solo satura el pitch ó 
             * ambos saturan y es mayor la ref de pitch.
             */
             rPhiComputadora = rThetaComputadora / thetaRoll;

        }
        else if((!satTheta & satRoll) | (satTheta & satRoll & (thetaRoll < 1))) 
        {
            /*
             * Si solo satura el roll ó
             * ambos saturan y es mayor la ref de roll.
             */
            rThetaComputadora = rPhiComputadora * thetaRoll;
        }

        // Eliminamos valores muy pequeños.
        rThetaComputadora = (rThetaComputadora < tol && rThetaComputadora > -tol) ? rThetaComputadora = 0f : rThetaComputadora; 
        rPhiComputadora = (rPhiComputadora < tol && rPhiComputadora > -tol) ? rPhiComputadora = 0f : rPhiComputadora; 

        // Actualizamos el error anterior.
        eaX = eX;
        eaZ = eZ;

        // Actualizamos referencia anterior.
        raX = rX;
        raZ = rZ;
    }

    //--------------------------------------------------------------------
    // Seteo de coordenadas automáticas para volver a la posición de Home.
    //--------------------------------------------------------------------
    private void OnRetornoHome()
    {
        if (automatico) // Si nos encontramos en el modo automático.
        {
            rX = rXHome;
            rZ = rZHome;
            rAlturaComputadora = rAlturaHome;
        }
    }

    //---------------------------------
    // Apertura cierre de pulverizador.
    //---------------------------------
    private void OnPulverizador()
    {
        pulverizador.flagApertura ^= true; // Cada vez que se llama el método, cambiamos el valor del flag. 
        pulverizador.AbrirCerrarPulverizador();
    }
}
