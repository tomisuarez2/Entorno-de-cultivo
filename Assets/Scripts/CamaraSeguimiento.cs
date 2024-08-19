using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//=====================================================
// Clase para crear una camara de seguimiento del dron.
//=====================================================
public class CamaraSeguimiento : MonoBehaviour
{
    //===========
    // Atributos.
    //===========

    private Transform dron; // Objeto de trasnformaci�n de posici�n y orientaci�n del dron.
    private Vector3 posicionTrasera = new Vector3(0,2,4); // Posici�n relativa de la camara respecto del dron.
    private Vector3 velocidadSeguimientoCamara; // Velocidad de seguimiento de la c�mara.
    private float angulo = 20; // �ngulo de la c�mara respecto del dron.

    //=========
    // M�todos.
    //=========

    void Awake()
    {
        dron = GameObject.FindGameObjectWithTag("Dron").transform; // Buscamos la trasnform del game object del dron.
    }

    void FixedUpdate()
    {
        transform.position = Vector3.SmoothDamp(transform.position, dron.transform.TransformPoint(posicionTrasera), ref velocidadSeguimientoCamara, 0.1f);
        transform.rotation = Quaternion.Euler(new Vector3(angulo, dron.GetComponent<IMU>().yaw * Mathf.Rad2Deg + 180, 0));
    }
}
