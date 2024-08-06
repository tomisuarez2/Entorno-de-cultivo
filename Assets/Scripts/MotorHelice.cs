using UnityEngine;

public class MotorHelice : MonoBehaviour
{
    //===========
    // Atributos.
    //===========

    private Transform posicion;            // Posici�n del conjunto.
    public int sentido;                    // Sentido de giro, -1 lev�giro, 1 dextr�giro.
    private float velocidadRotacionMinima; // Velocidad rotaci�n m�nima.
    public float velocidadRotacion;        // Velocidad de rotaci�n instantanea.

    //=========
    // M�todos.
    //=========

    void Awake()
    {
        // Velocidad m�nima de rotaci�n en rad/s.
        velocidadRotacionMinima = 30.45f;
        // Asignamos la transformaci�n del GameObject al atributo.
        posicion = transform;
    }

    public void RotacionConjunto(float deltaTime)
    {
        // Mantenemos una velocidad de rotacion m�nima, para evita el efecto visual de h�lices quietas.
        if(velocidadRotacion < velocidadRotacionMinima)
        {
            velocidadRotacion = velocidadRotacionMinima;
        }
        // M�todo para rotar la posici�n de la h�lice en funci�n de la velocidad.
        posicion.Rotate(Vector3.up, sentido * velocidadRotacion * Mathf.Rad2Deg * deltaTime);
    }
}
