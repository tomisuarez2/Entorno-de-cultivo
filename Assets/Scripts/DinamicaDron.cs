using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CalculoMatricial; // Clase creada para el calculo matricial.
using System;

public class DinamicaDron : MonoBehaviour
{
    //===========
    // Atributos.
    //===========

    // Clase correspondiente al controlador de vuelo.
    public ControladorDeVuelo controlador;

    // Atributos físicos.
    public Rigidbody rb;                   // Rigid body del dron.
    private float m = 0.225f;              // Masa del dron. [kg]
    private float g = 9.81f;               // Valor de la gravedad. [m/s^2]
    private float Ixx = 5e-3f;             // Momento de inercia del dron con respecto eje X, sistema B. [kg*m^2]
    private float Iyy = 5e-3f;             // Momento de inercia del dron con respecto eje Y, sistema B. [kg*m^2]
    private float Izz = 5e-3f;             // Momento de inercia del dron con respecto eje Z, sistema B. [kg*m^2]
    private float b_dot_x = 0.25f;         // Coeficiente de fricción en el X, sistema E. [N*s/m]
    private float b_dot_y = 0.25f;         // Coeficiente de fricción en el Y, sistema E. [N*s/m]
    private float b_dot_z = 0.25f;         // Coeficiente de fricción en el Z, sistema E. [N*s/m]
    private float[,] Mt;                   // Matriz de inercia traslacional.
    private float[,] invMt;                // Inversa de la matriz de inercia traslacional.
    private float[,] gt;                   // Vector de gravedad.
    private float[,] Mr;                   // Matriz de inercia rotacional.
    private float[,] invMr;                // Inversa de la matriz de inercia rotacional.
    private float[,] Cr;                   // Matriz de fuerzas centrífugas y de coríolis.
    private float[,] f;                    // Vector de fuerzas actuantes en el sistema de referencia E producto de la rotacion de motores.
    private float[,] tauB;                 // Vector de torques actuantes en el sistema de referencia B producto de la rotacion de motores.
    private float[,] ff;                   // Vector de fuerzas de fricción en el sistema de referencia E producto del movimiento del aparato.
    private float[,] tauf;                 // Vector de torques de fricción actuantes en el sistema de referencia B producto de la rotacion de motores.
    private float[,] f_netas;              // fuerzas netas traslacionales actuantes en el sistema E.
    private float[,] tau_netos;            // Torques netos actuantes en el sistema B.
    private float[,] dot_eta;              // Velocidades angulares del dron en el sistema de referencia B.
    private float[,] dotdot_xi;            // Aceleraciones traslacionales del dron en el sistema de referencia E.
    private float[,] dotdot_eta;           // Aceleraciones angulares del dron en el sistema de referencia B.

    // Vectores para aplicar aceleración sobre rigid body.
    private Vector3 aceleracionLineal;     // Vector para aceleraciones lineales, respecto de E.
    private Vector3 aceleracionAngular;    // Vector para aceleraciones angulares, respecto de B.

    // Tiempo de simulación.
    public float deltaTime;

    //=============================
    // Métodos Awake y FixedUpdate.
    //=============================

    void Awake()
    {
        Time.fixedDeltaTime = 0.001f;
        deltaTime = Time.fixedDeltaTime; // Tiempo de simulación.

        // Obtenemos el componente Rigidbody del dron.
        rb = GetComponent<Rigidbody>();

        // Seteamos la masa del mismo.
        rb.mass = m;

        // No usamos la gravedad del motor de física.
        rb.useGravity = false;

        // Inicializamos elementos constantes en matrices.

        Mt = new float[3, 3];
        Mt[0, 0] = m;
        Mt[0, 1] = 0f;
        Mt[0, 2] = 0f;
        Mt[1, 0] = 0f;
        Mt[1, 1] = m;
        Mt[1, 2] = 0f;
        Mt[2, 0] = 0f;
        Mt[2, 1] = 0f;
        Mt[2, 2] = m;

        invMt = new float[3, 3];
        invMt[0, 0] = 1 / m;
        invMt[0, 1] = 0f;
        invMt[0, 2] = 0f;
        invMt[1, 0] = 0f;
        invMt[1, 1] = 1 / m;
        invMt[1, 2] = 0f;
        invMt[2, 0] = 0f;
        invMt[2, 1] = 0f;
        invMt[2, 2] = 1 / m;

        gt = new float[3, 1];
        gt[0, 0] = 0f;
        gt[1, 0] = m * g;
        gt[2, 0] = 0f;

        Mr = new float[3, 3];

        invMr = new float[3, 3];

        Cr = new float[3, 3];

        f = new float[3, 1];

        tauB = new float[3, 1];

        ff = new float[3, 1];

        tauf = new float[3, 1];

        f_netas = new float[3, 1];

        tau_netos = new float[3, 1];

        dot_eta = new float[3, 1];

        dotdot_xi = new float[3, 1];

        dotdot_eta = new float[3, 1];
    }


    void FixedUpdate()
    {
        //-------------------------------
        // Actualización del controlador.
        //-------------------------------

        controlador.ComputarControlador(rb, transform, deltaTime, m, g);

        //------------------------------
        // Actualizacion de la dinámica.
        //------------------------------

        ActualizarDinamica();

        //--------------------------------------------------
        // Imprimimos dichas aceleraciones en el rigid body.
        //--------------------------------------------------

        rb.AddForce(aceleracionLineal, ForceMode.Acceleration);
        rb.AddRelativeTorque(aceleracionAngular, ForceMode.Acceleration);      
    }

    //=============================
    // Métodos propios de la clase.
    //=============================

    void ActualizarMr() // Método para actualizar los elementos de la matriz Mr.
    {
        float theta = controlador.imu.pitch; // Ángulo de pitch.
        float phi = controlador.imu.roll;    // Ángulo de roll.

        Mr[0, 0] = Ixx * Mathf.Cos(phi) * Mathf.Cos(phi) + Iyy * Mathf.Sin(phi) * Mathf.Sin(phi);
        Mr[0, 1] = 0.5f * (Ixx - Iyy) * Mathf.Cos(theta) * Mathf.Sin(2*phi);
        Mr[0, 2] = 0f;

        Mr[1, 0] = Mr[0, 1];
        Mr[1, 1] = Mathf.Cos(theta) * Mathf.Cos(theta) * (Ixx * Mathf.Sin(phi) * Mathf.Sin(phi) + Iyy * Mathf.Cos(phi) * Mathf.Cos(phi)) + Izz * Mathf.Sin(theta) * Mathf.Sin(theta);
        Mr[1, 2] = -Izz * Mathf.Sin(theta);

        Mr[2, 0] = Mr[0, 2];
        Mr[2, 1] = Mr[1, 2];
        Mr[2, 2] = Izz;
    }

    void ActualizarinvMr() 
    {
        /*
         * Podemos usar este método numérico y sino podemos probar usar la expresión
         * de la solución analítica.
         */
        invMr = OperacionesMatrices.CalcularInversa(Mr);
    }
    void ActualizarCr() // Método para actualizar los elementos de la matriz Cr.
    {
        float theta = controlador.imu.pitch;              // Ángulo de pitch.
        float phi = controlador.imu.roll;                 // Ángulo de roll.
        float thetaDot = controlador.imu.tasaCambioPitch; // Tasa de cambio ángulo de pitch.
        float phiDot = controlador.imu.tasaCambioRoll;    // Tasa de cambio ángulo de roll.
        float psiDot = controlador.imu.tasaCambioYaw;     // Tasa de cambio ángulo de yaw.

        Cr[0, 0] = phiDot * Mathf.Sin(2 * phi) * (Iyy - Ixx); 

        Cr[0, 1] = 0.5f * psiDot * Mathf.Sin(2*theta) * (Ixx * Mathf.Sin(phi) * Mathf.Sin(phi) + Iyy * Mathf.Cos(phi) * Mathf.Cos(phi) - Izz);

        Cr[0, 2] = psiDot * Mathf.Cos(theta) * ((Ixx - Iyy) * Mathf.Cos(phi) * Mathf.Cos(phi) + (Iyy - Ixx) * Mathf.Sin(phi) * Mathf.Sin(phi) + Izz);

        Cr[1, 0] = psiDot * Mathf.Sin(2 * theta) * (-Ixx * Mathf.Sin(phi) * Mathf.Sin(phi) - Iyy * Mathf.Cos(phi) * Mathf.Cos(phi) + Izz) + 0.5f * thetaDot * (Iyy - Ixx) * Mathf.Sin(2 * phi) * Mathf.Sin(theta);

        Cr[1, 1] = phiDot * Mathf.Sin(2 * phi) * Mathf.Cos(theta) * Mathf.Cos(theta) * (Ixx - Iyy) + thetaDot * Mathf.Sin(2 * theta) * Mathf.Cos(phi) * Mathf.Cos(phi) * (-Ixx - Iyy); 

        Cr[1, 2] = thetaDot * Mathf.Cos(theta) * ((Ixx - Iyy) * Mathf.Cos(phi) * Mathf.Cos(phi) + (Iyy - Ixx) * Mathf.Sin(phi) * Mathf.Sin(phi) - Izz) + psiDot * Mathf.Cos(theta) * Mathf.Cos(theta) * Mathf.Sin(2*phi) * (Ixx - Iyy);

        Cr[2, 0] = 0.5f * thetaDot * Mathf.Sin(2 * phi) * (Ixx - Iyy) + psiDot * Mathf.Cos(theta) * (2 * Mathf.Cos(phi) * Mathf.Cos(phi) * (Iyy - Ixx) + Ixx - Iyy - Izz);

        Cr[2, 1] = 0.5f * psiDot * Mathf.Cos(theta) * Mathf.Cos(theta) * Mathf.Sin(2 * phi) * (Iyy - Ixx);

        Cr[2, 2] = 0f;
    }

    void Actualizarf() // Método para actualizar el vector de fuerzas f (Referenciado a E).
    {
        float theta = controlador.imu.pitch;           // Ángulo de pitch.
        float phi = controlador.imu.roll;              // Ángulo de roll.
        float psi = controlador.imu.yaw;               // Ángulo de yaw.
        float Fy_B;                                    // Acción de control en B.

        Fy_B = controlador.accionEmpuje;
        f[0, 0] = (Mathf.Cos(phi) * Mathf.Sin(psi) * Mathf.Sin(theta) - Mathf.Cos(psi) * Mathf.Sin(phi)) * Fy_B;
        f[1, 0] = Mathf.Cos(phi) * Mathf.Cos(theta) * Fy_B;
        f[2, 0] = (Mathf.Sin(phi) * Mathf.Sin(psi) + Mathf.Cos(phi) * Mathf.Cos(psi) * Mathf.Sin(theta)) * Fy_B;
    }

    void Actualizartau() 
    {
        float theta = controlador.imu.pitch;           // Ángulo de pitch.
        float phi = controlador.imu.roll;              // Ángulo de roll.

        /*
         * Dado que las acciones de control de actitud se llevan a cabo en el sistema de referencia E 
         * y deben ser aplicadas en B en función de la dinámica derivada, debemos calcular la acción de control que habría en B,
         * para asi aplicarla al dron.
         */
        
        tauB[0, 0] = controlador.accionPitch * Mathf.Cos(phi) + controlador.accionYaw * Mathf.Cos(theta) * Mathf.Sin(phi);
        tauB[1, 0] = -controlador.accionPitch * Mathf.Sin(phi) + controlador.accionYaw * Mathf.Cos(theta) * Mathf.Cos(phi);
        tauB[2, 0] = -controlador.accionYaw * Mathf.Sin(phi) + controlador.accionRoll;

    }
    
    public void Actualizarff()
    { 
        ff[0, 0] = b_dot_x * controlador.imu.tasaCambioXB; 
        ff[1, 0] = b_dot_y * controlador.imu.tasaCambioYB; 
        ff[2, 0] = b_dot_z * controlador.imu.tasaCambioZB;
    }

    public void Actualizardot_eta()
    {
        dot_eta[0, 0] = controlador.imu.tasaCambioPitch;
        dot_eta[1, 0] = controlador.imu.tasaCambioYaw;
        dot_eta[2, 0] = controlador.imu.tasaCambioRoll;
    }

    //---------------------------------------------------------
    // Método para la actualización de la dinámica del sistema.
    //---------------------------------------------------------
    public void ActualizarDinamica()
    {
        ActualizarMr();
        ActualizarinvMr();
        ActualizarCr();
        Actualizardot_eta();

        //------------------------------------------------------
        // Actualización de las fuerzas ejercidas en el sistema.
        //------------------------------------------------------

        Actualizarf();
        Actualizartau();
        Actualizarff();

        //-------------------------------------------------
        // Cálculo de las fuerzas y torque netos actuantes.
        //-------------------------------------------------

        f_netas = OperacionesMatrices.SumaRestaResta(f, gt, ff);
        tau_netos = OperacionesMatrices.SumaRestaResta(tauB, OperacionesMatrices.Producto(Cr, dot_eta), tauf);

        //-------------------------------------------
        // Cálculo de las consecuentes aceleraciones.
        //-------------------------------------------

        dotdot_xi = OperacionesMatrices.Producto(invMt, f_netas);
        dotdot_eta = OperacionesMatrices.Producto(invMr, tau_netos);

        aceleracionLineal.x = dotdot_xi[0, 0];
        aceleracionLineal.y = dotdot_xi[1, 0];
        aceleracionLineal.z = dotdot_xi[2, 0];

        aceleracionAngular.x = dotdot_eta[0, 0] * Mathf.Cos(controlador.imu.roll) + dotdot_eta[1, 0] * Mathf.Cos(controlador.imu.pitch) * Mathf.Sin(controlador.imu.roll);
        aceleracionAngular.y = -dotdot_eta[0, 0] * Mathf.Sin(controlador.imu.roll) + dotdot_eta[1, 0] * Mathf.Cos(controlador.imu.pitch) * Mathf.Cos(controlador.imu.roll);
        aceleracionAngular.z = -dotdot_eta[1, 0] * Mathf.Sin(controlador.imu.pitch) + dotdot_eta[2, 0];

        aceleracionAngular *= Mathf.Rad2Deg; // Pasamos a grados, addTorque usa grados.
    }
}
