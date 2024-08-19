using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IMU : MonoBehaviour
{
    //===========
    // Atributos.
    //===========

    //----------------------------------------------------
    // Atributos para roll, pitch, yaw, posición y altura.
    //----------------------------------------------------

    [HideInInspector]
    public float roll;
    [HideInInspector]
    public float pitch;
    [HideInInspector]
    public float yaw;

    //-----------------
    // Tasas de cambio.
    //-----------------

    [HideInInspector]
    public float tasaCambioRoll;
    [HideInInspector]
    public float tasaCambioPitch;
    [HideInInspector]
    public float tasaCambioYaw;
    [HideInInspector]
    public float tasaCambioYB;
    [HideInInspector]
    public float tasaCambioXB;
    [HideInInspector]
    public float tasaCambioZB;

    //=========
    // Métodos.
    //=========

    public void ActualizacionGiroscopo(Rigidbody rb,Transform objetoTransformacion)
    {
        if (objetoTransformacion == null) return;

        // Obtenemos la rotación del objeto en ángulos de Euler. Nos colocamos en rango [-pi,pi].
        Vector3 rotacionEuler = objetoTransformacion.eulerAngles;
        rotacionEuler.x = rotacionEuler.x > 180 ? rotacionEuler.x - 360 : (rotacionEuler.x < -180 ? rotacionEuler.x + 360 : rotacionEuler.x);
        rotacionEuler.y = rotacionEuler.y > 180 ? rotacionEuler.y - 360 : (rotacionEuler.y < -180 ? rotacionEuler.y + 360 : rotacionEuler.y);
        rotacionEuler.z = rotacionEuler.z > 180 ? rotacionEuler.z - 360 : (rotacionEuler.z < -180 ? rotacionEuler.z + 360 : rotacionEuler.z);

        /* 
           Obtenemos la velocidad de rotación del objeto en ángulos de Euler.
           El atributo angularVelocity mide wx, wy y wz. No mide tasa de cambio de ángulos de Euler.
           Según dinámica:
        */
        
        tasaCambioPitch = rb.angularVelocity.x * Mathf.Cos(yaw) - rb.angularVelocity.z * Mathf.Sin(yaw);
        tasaCambioRoll = rb.angularVelocity.x * Mathf.Sin(yaw) * Mathf.Tan(pitch) + rb.angularVelocity.y + rb.angularVelocity.z * Mathf.Cos(yaw) * Mathf.Tan(pitch);
        tasaCambioYaw = rb.angularVelocity.x * Mathf.Sin(yaw) / Mathf.Cos(pitch) + rb.angularVelocity.z * Mathf.Cos(yaw) / Mathf.Cos(pitch);

        tasaCambioYB = rb.velocity.y;
        tasaCambioXB = rb.velocity.x; // Esto se supone debe ser en B, lo dejamos en E.
        tasaCambioZB = rb.velocity.z;

        // Actualizamos los valores de roll, pitch, yaw y altura en Y.
        roll = rotacionEuler.z * Mathf.Deg2Rad;
        pitch = rotacionEuler.x * Mathf.Deg2Rad;
        yaw = rotacionEuler.y * Mathf.Deg2Rad;
    }
}
