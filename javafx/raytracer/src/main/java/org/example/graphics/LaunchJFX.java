package org.example.graphics;

import javafx.application.Application;

/**
 * Вспомогательный класс для запуска приложения JavaFX без ручного добавления модулей и экспорта в командную строку Java.
 */
public class LaunchJFX {
   public static void main( final String[] args ) throws ClassNotFoundException {
      System.setProperty( "javafx.animation.fullspeed", "true" );
      if ( args.length < 1 || args[ 0 ].contains( LaunchJFX.class.getName() ) ) {
         Application.launch( TinyRayTracerFX.class );
      } else {
         Class< ? extends Application > clazz = Class.forName( args[ 0 ] ).asSubclass( Application.class );
         if ( Application.class.isAssignableFrom( clazz ) ) {
            Application.launch( clazz );
         } else {
            Application.launch( TinyRayTracerFX.class );
         }
      }
   }
}