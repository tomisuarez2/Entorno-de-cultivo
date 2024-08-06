using UnityEngine;

public class MotorHelice : MonoBehaviour
{
    //===========
    // Atributos.
    //===========

    private Transform posicion;            // Posición del conjunto.
    public int sentido;                    // Sentido de giro, -1 levógiro, 1 dextrógiro.
    private float velocidadRotacionMinima; // Velocidad rotación mínima.
    public float velocidadRotacion;        // Velocidad de rotación instantanea.

    //=========
    // Métodos.
    //=========

    void Awake()
    {
        // Velocidad mínima de rotación en rad/s.
        velocidadRotacionMinima = 30.45f;
        // Asignamos la transformación del GameObject al atributo.
        posicion = transform;
    }

    public void RotacionConjunto(float deltaTime)
    {
        // Mantenemos una velocidad de rotacion mínima, para evita el efecto visual de hélices quietas.
        if(velocidadRotacion < velocidadRotacionMinima)
        {
            velocidadRotacion = velocidadRotacionMinima;
        }
        // Método para rotar la posición de la hélice en función de la velocidad.
        posicion.Rotate(Vector3.up, sentido * velocidadRotacion * Mathf.Rad2Deg * deltaTime);
    }
}
