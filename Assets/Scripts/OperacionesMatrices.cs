using System;
using UnityEngine;
using static TMPro.SpriteAssetUtilities.TexturePacker_JsonArray;

//==============================================
// Clase que realiza operaciones entre matrices.
//==============================================

namespace CalculoMatricial
{
    public class OperacionesMatrices : MonoBehaviour 
    {
        //--------------------------
        // Producto matricial (A*B).
        //--------------------------
        public static float[,] Producto(float[,] matrizA, float[,] matrizB)
        {
            int aFilas = matrizA.GetLength(0);
            int aColumnas = matrizA.GetLength(1);
            int bFilas = matrizB.GetLength(0);
            int bColumnas = matrizB.GetLength(1);

            if (aColumnas != bFilas)
                throw new InvalidOperationException("El número de columnas de la primera matriz debe ser igual al número de filas de la segunda matriz.");

            float[,] resultado = new float[aFilas, bColumnas];

            for (int i = 0; i < aFilas; i++)
            {
                for (int j = 0; j < bColumnas; j++)
                {
                    for (int k = 0; k < aColumnas; k++)
                    {
                        resultado[i, j] += matrizA[i, k] * matrizB[k, j];
                    }
                }
            }

            return resultado;
        }

        //------------------------------
        // Suma de matrices (A - B - C).
        //------------------------------
        public static float[,] SumaRestaResta(float[,] matrizA, float[,] matrizB, float[,] matrizC)
        {
            int filas = matrizA.GetLength(0);
            int columnas = matrizA.GetLength(1);

            if (filas != matrizB.GetLength(0) || columnas != matrizB.GetLength(1) || filas != matrizC.GetLength(0) || columnas != matrizC.GetLength(1))
                throw new InvalidOperationException("Las matrices deben tener las mismas dimensiones.");

            float[,] resultado = new float[filas, columnas];

            for (int i = 0; i < filas; i++)
            {
                for (int j = 0; j < columnas; j++)
                {
                    resultado[i, j] = matrizA[i, j] - matrizB[i, j] - matrizC[i, j];
                }
            }

            return resultado;
        }

        //-------------------------------------
        // Cálculo de la inversa de una matriz.
        //-------------------------------------

        public static float[,] CalcularInversa(float[,] matriz)
        {
            int n = matriz.GetLength(0);
            float[,] identidad = CrearMatrizIdentidad(n);
            float[,] augmentada = AugmentarMatrices(matriz, identidad);

            for (int i = 0; i < n; i++)
            {
                if (augmentada[i, i] == 0)
                {
                    // Intercambiamos filas si el elemento diagonal es cero.
                    if (!IntercambiarFilas(augmentada, i))
                    {
                        return null; // No se puede invertir si no se puede intercambiar filas.
                    }
                }

                // Escalamos fila para que el elemento diagonal sea 1.
                float factor = augmentada[i, i];
                for (int j = 0; j < 2 * n; j++)
                {
                    augmentada[i, j] /= factor;
                }

                // Hacemos ceros en la columna i para todas las demás filas.
                for (int k = 0; k < n; k++)
                {
                    if (k != i)
                    {
                        factor = augmentada[k, i];
                        for (int j = 0; j < 2 * n; j++)
                        {
                            augmentada[k, j] -= factor * augmentada[i, j];
                        }
                    }
                }
            }

            // Extraemos la parte derecha de la matriz augmentada como la inversa.
            float[,] inversa = new float[n, n];
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    inversa[i, j] = augmentada[i, j + n];
                }
            }

            return inversa;
        }

        //--------------------------------------------------------
        // Creación de una matriz identidad de tamaño determinado.
        //--------------------------------------------------------

        public static float[,] CrearMatrizIdentidad(int tamano)
        {
            float[,] identidad = new float[tamano, tamano];
            for (int i = 0; i < tamano; i++)
            {
                identidad[i, i] = 1;
            }
            return identidad;
        }

        //----------------------------------
        // Creación de la matriz augmentada.
        //----------------------------------

        public static float[,] AugmentarMatrices(float[,] A, float[,] B)
        {
            int filas = A.GetLength(0);
            int columnas = A.GetLength(1);
            float[,] augmentada = new float[filas, 2 * columnas];

            for (int i = 0; i < filas; i++)
            {
                for (int j = 0; j < columnas; j++)
                {
                    augmentada[i, j] = A[i, j];
                    augmentada[i, j + columnas] = B[i, j];
                }
            }
            return augmentada;
        }

        //----------------------------------------------
        // Intercambio de filas en la matriz augmentada.
        //----------------------------------------------

        public static bool IntercambiarFilas(float[,] matriz, int fila)
        {
            int n = matriz.GetLength(0);
            for (int i = fila + 1; i < n; i++)
            {
                if (matriz[i, fila] != 0)
                {
                    for (int j = 0; j < 2 * n; j++)
                    {
                        float auxiliar = matriz[fila, j];
                        matriz[fila, j] = matriz[i, j];
                        matriz[i, j] = auxiliar;
                    }
                    return true;
                }
            }
            return false;
        }
    }
}